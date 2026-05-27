# docs/src/extensions/rouge_ini.rb
#
# This fixes the default Rouge lexer for INI files to include comments with
# leading '#' which are not followed by an emtpy line.
#
# Loaded via the asciidoctor -r flag from the docs Submakefile.

require "rouge"

module Rouge
  module Lexers
    class INI < RegexLexer
      title "INI"
      desc "Fixed INI lexer for # comments"

      state :basic do
        rule %r/^[ \t]*[;#][^\n]*(?=\n|\z)/, Comment
      end
    end
  end
end
