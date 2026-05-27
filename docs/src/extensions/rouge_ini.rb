# docs/src/extensions/rouge_ini.rb
#
# Fixes Rouge's default INI lexer for LinuxCNC INI files:
#   - accepts `#`/`;` comments without a trailing newline (end of file/block),
#   - accepts leading whitespace before `key = value` lines,
#   - highlights the `#INCLUDE` directive as a preprocessor token.
#
# Reopening `state :basic do` would replace the upstream rules and lose the
# whitespace handler, so we rebuild it explicitly here.
#
# Loaded via the asciidoctor -r flag from the docs Submakefile.

require "rouge"

module Rouge
  module Lexers
    class INI < RegexLexer
      title "INI"
      desc "INI with LinuxCNC #INCLUDE + tolerant comments / whitespace"

      state :basic do
        rule %r/(^#INCLUDE)([ \t]+)([^\n]+)/ do
          groups Comment::Preproc, Text, Str
        end
        rule %r/[;#].*?(?=\n|\z)/, Comment
        rule %r/\s+/, Text
        rule %r/\\\n/, Str::Escape
      end
    end
  end
end
