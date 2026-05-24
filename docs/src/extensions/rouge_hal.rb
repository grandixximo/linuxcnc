# docs/src/extensions/rouge_hal.rb
#
# Rouge lexer for LinuxCNC HAL (Hardware Abstraction Layer) script files.
# Ported from docs/src/source-highlight/hal.lang (Michael Haberler, 2011).
#
# Highlights:
#   - halcmd commands (loadrt, net, setp, addf, ...)
#   - pin / signal names of the form  component.pin-name
#   - INI substitutions of the form   [SECTION]NAME
#   - environment variables           $VAR  $(VAR)
#   - assignment operators            =  =>  <=
#   - numbers and comments
#
# Loaded via the asciidoctor -r flag from the docs Submakefile.

require 'rouge'

module Rouge
  module Lexers
    class HAL < RegexLexer
      title 'HAL'
      desc 'LinuxCNC Hardware Abstraction Layer (halcmd)'
      tag 'hal'
      filenames '*.hal', '*.tcl.hal', '*.postgui.hal'
      mimetypes 'text/x-hal'

      COMMANDS = %w[
        loadrt unloadrt loadusr waitusr unloadusr
        newsig delsig sets gets stype ptype getp setp
        linkps linksp linkpp net unlinkp
        addf delf start stop
        show item save source
        alias unalias list
        lock unlock status help
        echo
      ].join('|').freeze

      state :root do
        rule %r/\s+/, Text
        rule %r/#.*$/, Comment::Single

        # Commands as the first non-space token of a line
        rule %r/(?<=^|\s)(?:#{COMMANDS})\b/i, Keyword

        # Assignment operators
        rule %r/=>|<=|=/, Operator

        # INI substitution: [SECTION]NAME
        rule %r/\[[A-Z_][A-Z0-9_]*\][A-Z_][A-Z0-9_]*/i, Name::Constant

        # Environment variables
        rule %r/\$\([A-Z_][A-Z0-9_]*\)/i, Name::Variable
        rule %r/\$[A-Z_][A-Z0-9_]*/i, Name::Variable

        # Pin/signal names: token with at least one dot
        rule %r/[A-Za-z_][\w-]*(?:\.[\w-]+)+/, Name::Attribute

        # Numbers (signed, optional decimal, optional exponent)
        rule %r/[+-]?\d+\.\d+(?:[eE][+-]?\d+)?/, Num::Float
        rule %r/[+-]?\d+(?:[eE][+-]?\d+)?/, Num::Integer

        # Quoted strings (rare in HAL but used in echo / save args)
        rule %r/"[^"\n]*"/, Str::Double
        rule %r/'[^'\n]*'/, Str::Single

        # Bare token: signal or component name without dots
        rule %r/[A-Za-z_][\w-]*/, Name

        # Anything else
        rule %r/./, Text
      end
    end
  end
end
