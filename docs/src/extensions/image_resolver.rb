# docs/src/extensions/image_resolver.rb
#
# Asciidoctor treeprocessor that resolves image targets the way the
# asciidoc-py + docs/src/image-wildcard pair did:
#   - relative image paths in an included file are resolved relative to
#     that included file's directory (not the top-level master)
#   - missing extension is filled in by trying png/svg/jpg/jpeg
#
# Requires the document to be loaded with sourcemap: true (passed on the
# CLI as -a sourcemap=true) so blocks expose .file.

require 'asciidoctor'
require 'asciidoctor/extensions'

module LinuxCNCDocs
  class ImageResolver < Asciidoctor::Extensions::Treeprocessor
    IMAGE_EXTS = %w[.png .svg .jpg .jpeg].freeze
    # Inline image: macros, with target as captured group 1.  Skip the
    # block image:: form (handled by find_by(:image)) and anything that
    # already looks like a URL or absolute path.
    INLINE_IMAGE_RE = /(?<![:\w])image:(?!:)([^\[\s]+)\[/
    # Language subdirectories of docs/src/.  Stripped from the path when
    # falling back to the English source tree for images that exist only
    # in the canonical location.
    LANG_RE = %r{/src/(de|es|fr|nb|ru|uk|zh_CN)/}

    def process(document)
      # For HTML output the relative target is correct as-is — asciidoctor
      # writes it straight into <img src=...> and a sibling copy of the
      # image tree makes it resolve at deploy time. Only PDF embedding
      # needs absolute paths so prawn-svg / image reading can find the
      # file at convert time.
      return document unless document.backend == 'pdf'
      walk(document)
      document
    end

    # asciidoctor-style table cells (cols="...a") parse their content as
    # an inner Document, so find_by on the master doesn't reach blocks
    # nested inside them.  Walk into each cell's inner_document too.
    def walk(doc)
      doc.find_by(context: :image) { |n| rewrite n }
      # Inline image: macros are part of block text and never appear as
      # standalone nodes in find_by; scan block-level source storage for
      # them.  Asciidoctor parks block text in :lines (literal/paragraph)
      # or :text (list_item).
      doc.find_by do |b|
        next unless b.file
        rewrite_inline_in_block(b)
        false
      end
      doc.find_by(context: :table_cell) do |c|
        inner = c.inner_document if c.respond_to?(:inner_document)
        walk(inner) if inner
      end
    end

    def rewrite(node)
      target = node.attr 'target'
      return unless target
      return if target.empty?
      return if target.start_with?('http://', 'https://', '/')
      return if target.include?('{') # leave macros alone

      src = node.file || node.document.attr('docfile')
      return unless src
      base_dir = File.dirname(File.expand_path(src))

      candidate = resolve_candidate(File.expand_path(target, base_dir))
      return unless candidate

      node.set_attr('target', candidate)
      apply_default_width(node)
    end

    # Try the requested path; fall back to the canonical English source
    # if the request points into a translated tree.  Image directories
    # under docs/src/<lang>/ are not always populated for every macro a
    # translated file references, but the English original at
    # docs/src/.../ usually exists.
    def resolve_candidate(path)
      probe = ->(p) {
        return p if File.file?(p)
        r = resolve_extension(p)
        return r if r && File.file?(r)
        nil
      }
      r = probe.call(path)
      return r if r
      fallback = path.sub(LANG_RE, '/src/')
      fallback != path ? probe.call(fallback) : nil
    end

    def rewrite_inline_in_block(block)
      base_dir = File.dirname(File.expand_path(block.file))

      # :paragraph / :literal / :sidebar etc. carry source in .lines (Array<String>).
      if block.respond_to?(:lines=) && block.lines.is_a?(Array) && !block.lines.empty?
        changed = false
        new_lines = block.lines.map { |ln| rewrite_inline(ln, base_dir) { changed = true } }
        block.lines = new_lines if changed
      end

      # :list_item carries source in .text (String).
      if block.respond_to?(:text=) && block.instance_variable_defined?(:@text)
        old = block.instance_variable_get(:@text)
        if old.is_a?(String) && !old.empty?
          changed = false
          new_text = rewrite_inline(old, base_dir) { changed = true }
          block.text = new_text if changed
        end
      end
    end

    def rewrite_inline(text, base_dir)
      text.gsub(INLINE_IMAGE_RE) do
        full = Regexp.last_match(0)
        target = Regexp.last_match(1)
        next full if target.start_with?('http://', 'https://', '/')
        next full if target.include?('{')

        candidate = resolve_candidate(File.expand_path(target, base_dir))
        if candidate
          yield if block_given?
          "image:#{candidate}["
        else
          full
        end
      end
    end

    # asciidoctor-pdf renders raster images at native pixel dimensions
    # interpreted as 72 DPI, then caps at content width.  Most of our
    # source PNGs are screenshots/diagrams sized for ~150 DPI display, so
    # the default behaviour blows them up to full text column width and
    # leaves big half-blank pages where they break across a page boundary.
    # dblatex defaulted to a smaller fit.  Approximate that by setting a
    # default pdfwidth when the source did not pin width/scaledwidth/pdfwidth.
    def apply_default_width(node)
      return if node.context == :inline_image
      return if node.attr('pdfwidth')
      return if node.attr('scaledwidth')
      return if node.attr('width')
      node.set_attr('pdfwidth', '75%')
    end

    def resolve_extension(path)
      return path if File.file?(path)
      return nil unless File.extname(path).empty?
      IMAGE_EXTS.each do |e|
        c = path + e
        return c if File.file?(c)
      end
      nil
    end
  end
end

Asciidoctor::Extensions.register do
  treeprocessor LinuxCNCDocs::ImageResolver
end
