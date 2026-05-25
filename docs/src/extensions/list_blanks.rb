# docs/src/extensions/list_blanks.rb
#
# Asciidoctor preprocessor that inserts a blank line before a bullet-list
# marker when it follows a non-empty, non-list line.  Asciidoctor strictly
# requires the blank delimiter; asciidoc-py was lenient, and the LinuxCNC
# docs were authored against the lenient behaviour.  Rather than touching
# 200+ files we adjust the source stream at parse time so blame and history
# stay clean.
#
# Only touches the top-level list marker `* `.  Bullets inside listing,
# literal, passthrough, comment and table blocks are left alone so we
# don't corrupt verbatim content.
#
# Wire-up (in Submakefile):
#   asciidoctor -r $(DOC_SRCDIR)/extensions/list_blanks.rb ...

require 'asciidoctor'
require 'asciidoctor/extensions'

module LinuxCNCDocs
  class ListBlanks < Asciidoctor::Extensions::Preprocessor
    # Bullet at column 0 (asciidoctor allows up to '*****' for nesting,
    # but only the column-0 ones are paragraph-breaking and only those
    # are the broken pattern in the LinuxCNC docs).
    BULLET = /\A\*\s/

    # Delimited block fences we must NOT enter.  Pair-matched on the
    # *exact* delimiter length so a closing fence matches its opener.
    FENCE = /\A(-{4,}|\.{4,}|\++|\/{4,}|\|={3,})\s*\z/

    def process(document, reader)
      lines = reader.lines
      out = []
      inside = nil # nil or the opening fence string we're waiting to close

      lines.each_with_index do |line, i|
        if inside
          out << line
          inside = nil if line.strip == inside
          next
        end

        if (m = FENCE.match(line))
          inside = m[1]
          out << line
          next
        end

        if BULLET.match?(line)
          prev = out.last
          if prev && !prev.empty? && !BULLET.match?(prev) && !prev.start_with?('//')
            # The previous line is a paragraph line; asciidoctor would
            # glue this bullet onto it.  Inject a blank so the list
            # opens.
            out << ''
          end
        end

        out << line
      end

      Asciidoctor::PreprocessorReader.new(document, out, reader.cursor)
    end
  end
end

Asciidoctor::Extensions.register do
  preprocessor LinuxCNCDocs::ListBlanks
end
