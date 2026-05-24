# docs/src/extensions/rouge_ngc.rb
#
# Rouge lexer for LinuxCNC NGC (RS-274/EMC dialect G-code) files.
# Ported from docs/src/source-highlight/ngc.lang (Michael Haberler 2011,
# originally adapted from Jan Van Gilsen's gedit highlight definition).
#
# Highlights:
#   - line numbers           N123
#   - G / M / T / H / F / S codes
#   - axis & argument letters X Y Z A B C U V W I J K P Q R L
#   - numeric, named, and indexed parameters  #1  #5421  #<varname>
#   - O-word blocks with their keywords (sub, while, if, call, ...)
#   - math functions and boolean operators
#   - comments: ( ... )   ; rest-of-line   (DEBUG, ...)
#
# Loaded via the asciidoctor -r flag from the docs Submakefile.

require 'rouge'

module Rouge
  module Lexers
    class NGC < RegexLexer
      title 'NGC'
      desc 'RS-274 G-code, LinuxCNC dialect'
      tag 'ngc'
      aliases 'gcode', 'rs274ngc'
      filenames '*.ngc', '*.nc', '*.tap'
      mimetypes 'text/x-ngc'

      MATH = %w[
        cos sin tan acos asin atan
        exp ln sqrt
        fup fix round
        abs exists
      ].join('|').freeze

      BOOL = %w[and or xor not mod gt lt ge le eq ne].join('|').freeze

      OWORD = %w[
        sub endsub return
        if elseif else endif
        while endwhile do break continue
        repeat endrepeat
        call
      ].join('|').freeze

      state :root do
        rule %r/\s+/, Text

        # Line numbers (N123 at start of statement)
        rule %r/(?<=^|\s)[nN]\d+/, Comment::Preproc

        # Comments
        rule %r/;.*$/, Comment::Single
        rule %r/\([dD][eE][bB][uU][gG],/, Comment::Special, :debug_comment
        rule %r/\([^)]*\)/, Comment::Multiline

        # Parameters
        rule %r/#<[^>]+>/, Name::Variable
        rule %r/#\d+/, Name::Variable

        # O-word lines: O<name> sub  or  O100 if
        rule %r/(?i:[oO])(?:\d+|<[A-Za-z_][\w-]*>)/, Name::Label
        rule %r/(?i:#{OWORD})\b/, Keyword::Reserved

        # Math + boolean operator names
        rule %r/(?i:#{MATH})\b/, Name::Builtin
        rule %r/(?i:#{BOOL})\b/, Operator::Word

        # G / M codes (G33.1, G92.2, M62, ...)
        rule %r/(?i:g)\d+(?:\.\d+)?/, Keyword
        rule %r/(?i:m)\d+/, Keyword

        # T H F S codes
        rule %r/(?i:[tfsh])-?\d+(?:\.\d+)?/, Keyword

        # Axis letters and argument letters followed by a value or expression
        rule %r/(?i:[xyzabcuvwijkpqrle])(?=\s*[-+\[\d#])/, Name::Attribute

        # Arithmetic / brackets
        rule %r{[+\-*/=]}, Operator
        rule %r/[\[\]]/, Punctuation

        # Numbers
        rule %r/[+-]?\d+\.\d*(?:[eE][+-]?\d+)?/, Num::Float
        rule %r/[+-]?\.\d+(?:[eE][+-]?\d+)?/, Num::Float
        rule %r/[+-]?\d+/, Num::Integer

        rule %r/./, Text
      end

      # (DEBUG, ...): same as a comment, but parameter references inside
      # should still be highlighted as variables (DEBUG expands them at
      # run time).
      state :debug_comment do
        rule %r/\)/, Comment::Special, :pop!
        rule %r/#<[^>]+>/, Name::Variable
        rule %r/#\d+/, Name::Variable
        rule %r/[^)#]+/, Comment::Special
      end
    end
  end
end
