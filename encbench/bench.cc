// Encoding comparison on captured EmcStat frames:
// FlatBuffers (rene's websocket-wip schema) vs JSON full snapshot vs JSON
// merge-patch deltas (RFC 7386), plus CBOR of the delta.
#include <chrono>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <string>
#include <vector>
#include <algorithm>
#include "tojson.hh"
#include "applyjson.hh"

using json = nlohmann::json;
using clk = std::chrono::steady_clock;

static std::vector<std::vector<uint8_t>> load(const char *path)
{
    std::vector<std::vector<uint8_t>> v;
    std::ifstream f(path, std::ios::binary);
    uint32_t n;
    while (f.read(reinterpret_cast<char *>(&n), 4)) {
        std::vector<uint8_t> b(n);
        f.read(reinterpret_cast<char *>(b.data()), n);
        v.push_back(std::move(b));
    }
    return v;
}

// RFC 7386 diff: object members recurse, everything else (arrays too) is
// replaced whole, removed members become null.
static json merge_diff(const json &a, const json &b)
{
    if (!a.is_object() || !b.is_object()) return b;
    json d = json::object();
    for (auto it = b.begin(); it != b.end(); ++it) {
        auto ai = a.find(it.key());
        if (ai == a.end()) { d[it.key()] = it.value(); continue; }
        if (*ai == it.value()) continue;
        if (ai->is_object() && it.value().is_object()) {
            json s = merge_diff(*ai, it.value());
            if (!s.empty()) d[it.key()] = std::move(s);
        } else {
            d[it.key()] = it.value();
        }
    }
    for (auto it = a.begin(); it != a.end(); ++it)
        if (!b.contains(it.key())) d[it.key()] = nullptr;
    return d;
}

// Variant: arrays of objects become objects keyed by index, so a delta
// carries only the joint/axis/spindle entries and fields that changed.
static json index_arrays(const json &j)
{
    if (j.is_object()) {
        json o = json::object();
        for (auto it = j.begin(); it != j.end(); ++it) o[it.key()] = index_arrays(it.value());
        return o;
    }
    if (j.is_array() && !j.empty() && j[0].is_object()) {
        json o = json::object();
        for (size_t i = 0; i < j.size(); ++i) o[std::to_string(i)] = index_arrays(j[i]);
        return o;
    }
    return j;
}

struct Acc { double ns = 0; size_t n = 0; void add(clk::duration d) { ns += std::chrono::duration<double, std::nano>(d).count(); ++n; } double avg() const { return n ? ns / n : 0; } };
struct Sz { double sum = 0; size_t n = 0, mx = 0; void add(size_t s) { sum += s; ++n; mx = std::max(mx, s); } double avg() const { return n ? sum / n : 0; } };

int main(int argc, char **argv)
{
    if (argc < 2) { fprintf(stderr, "usage: bench frames.bin [delta_out_prefix]\n"); return 1; }
    auto frames = load(argv[1]);
    fprintf(stderr, "%zu frames\n", frames.size());
    std::vector<EMC::EmcStatT> objs(frames.size());
    for (size_t i = 0; i < frames.size(); ++i) {
        flatbuffers::Verifier v(frames[i].data(), frames[i].size());
        if (!EMC::VerifyEmcStatBuffer(v)) { fprintf(stderr, "bad frame %zu\n", i); return 1; }
        EMC::GetEmcStat(frames[i].data())->UnPackTo(&objs[i]);
    }

    for (size_t step : {1, 10, 100}) {
        Sz s_fbs, s_full, s_delta, s_delta_ix, s_cbor_ix, s_patch6902;
        Acc e_fbs, e_full, e_delta_ix, e_fdiff, d_fbs_verify, d_fbs_unpack, d_full, d_delta_ix, d_simd;
        EMC::EmcStatT sstate; simdjson::ondemand::parser sj;
        flatbuffers::FlatBufferBuilder fbb(8192);
        json prev, prev_ix, state_ix;
        size_t zero_deltas = 0, updates = 0, fdiff_mismatch = 0; Sz s_fdiff; size_t prev_i = 0;
        std::ofstream dump;
        if (argc > 2) dump.open(std::string(argv[2]) + "_" + std::to_string(step) + ".jsonl");
        for (size_t i = 0; i < objs.size(); i += step) {
            ++updates;
            // FlatBuffers encode from the native object (stands in for EMC_STAT).
            auto t0 = clk::now();
            fbb.Clear();
            fbb.Finish(EMC::EmcStat::Pack(fbb, &objs[i]));
            auto t1 = clk::now();
            e_fbs.add(t1 - t0);
            s_fbs.add(fbb.GetSize());
            std::vector<uint8_t> buf(fbb.GetBufferPointer(), fbb.GetBufferPointer() + fbb.GetSize());

            // FlatBuffers decode: verify only (zero-copy access) and full unpack.
            t0 = clk::now();
            flatbuffers::Verifier ver(buf.data(), buf.size());
            bool ok = EMC::VerifyEmcStatBuffer(ver);
            t1 = clk::now();
            d_fbs_verify.add(t1 - t0);
            if (!ok) return 2;
            EMC::EmcStatT back;
            t0 = clk::now();
            EMC::GetEmcStat(buf.data())->UnPackTo(&back);
            t1 = clk::now();
            d_fbs_unpack.add(t1 - t0);

            // JSON full snapshot.
            t0 = clk::now();
            json cur; to_json(cur, objs[i]);
            std::string full = cur.dump();
            t1 = clk::now();
            e_full.add(t1 - t0);
            s_full.add(full.size());
            t0 = clk::now();
            json parsed = json::parse(full);
            t1 = clk::now();
            d_full.add(t1 - t0);

            // JSON merge-patch delta, plain RFC 7386 (arrays whole).
            if (!prev.is_null()) {
                json d = merge_diff(prev, cur);
                s_delta.add(d.dump().size());
                s_patch6902.add(json::diff(prev, cur).dump().size());
            }
            // Index-keyed variant: the encoder that would actually be used.
            t0 = clk::now();
            json cur2; to_json(cur2, objs[i]);
            json cur_ix = index_arrays(cur2);
            std::string dtxt;
            if (prev_ix.is_null()) dtxt = cur_ix.dump();
            else { json d = merge_diff(prev_ix, cur_ix); dtxt = d.dump(); if (d.empty()) ++zero_deltas; }
            t1 = clk::now();
            if (!prev_ix.is_null()) {
                e_delta_ix.add(t1 - t0);
                auto f0 = clk::now();
                json fd = json::object();
                diff_json(fd, objs[prev_i], objs[i]);
                std::string ftxt = fd.dump();
                auto f1 = clk::now();
                e_fdiff.add(f1 - f0);
                s_fdiff.add(ftxt.size());
                if (json::parse(ftxt) != json::parse(dtxt)) ++fdiff_mismatch;
                s_delta_ix.add(dtxt.size());
                json d = json::parse(dtxt);
                s_cbor_ix.add(json::to_cbor(d).size());
                t0 = clk::now();
                json dp = json::parse(dtxt);
                state_ix.merge_patch(dp);
                t1 = clk::now();
                d_delta_ix.add(t1 - t0);
                // Generated decoder: simdjson on-demand straight into the
                // client's native status struct, no DOM, no merge.
                t0 = clk::now();
                simdjson::padded_string ps(dtxt);
                auto doc = sj.iterate(ps);
                EMC::apply_json(doc.get_object(), sstate);
                t1 = clk::now();
                d_simd.add(t1 - t0);
            } else {
                state_ix = json::parse(dtxt);
                sstate = objs[i];
            }
            if (dump.is_open()) dump << dtxt << '\n';
            prev_i = i;
            prev = std::move(cur);
            prev_ix = std::move(cur_ix);
        }
        if (state_ix != prev_ix) { fprintf(stderr, "delta replay mismatch at step %zu\n", step); return 3; }
        { json a, b; to_json(a, sstate); to_json(b, objs[prev_i]);
          if (a != b) { fprintf(stderr, "simdjson apply mismatch at step %zu\n", step); return 4; } }
        printf("\n== every %zu frame(s): %zu updates, %zu empty deltas\n", step, updates, zero_deltas);
        printf("bytes/update   flatbuffers %8.0f (max %zu)\n", s_fbs.avg(), s_fbs.mx);
        printf("               json full   %8.0f (max %zu)\n", s_full.avg(), s_full.mx);
        printf("               merge-patch %8.0f (max %zu)   arrays whole\n", s_delta.avg(), s_delta.mx);
        printf("               merge-patch %8.0f (max %zu)   arrays by index\n", s_delta_ix.avg(), s_delta_ix.mx);
        printf("               same, CBOR  %8.0f (max %zu)\n", s_cbor_ix.avg(), s_cbor_ix.mx);
        printf("               RFC 6902    %8.0f (max %zu)\n", s_patch6902.avg(), s_patch6902.mx);
        printf("encode us      flatbuffers %8.2f\n", e_fbs.avg() / 1000);
        printf("               json full   %8.2f\n", e_full.avg() / 1000);
        printf("               json delta  %8.2f   (to_json + diff + dump)\n", e_delta_ix.avg() / 1000);
        printf("               json fdiff  %8.2f   (generated field diff + dump; %zu mismatches, %.0f B)\n", e_fdiff.avg() / 1000, fdiff_mismatch, s_fdiff.avg());
        printf("decode us      fbs verify  %8.2f   (then zero-copy reads)\n", d_fbs_verify.avg() / 1000);
        printf("               fbs unpack  %8.2f\n", d_fbs_unpack.avg() / 1000);
        printf("               json full   %8.2f\n", d_full.avg() / 1000);
        printf("               json delta  %8.2f   (nlohmann parse + merge_patch)\n", d_delta_ix.avg() / 1000);
        printf("               json delta  %8.2f   (simdjson on-demand into native struct)\n", d_simd.avg() / 1000);
    }
    return 0;
}
