# docs/src/extensions/rouge_ini.rb
#
# This extends the default Rouge lexer for INI files to include comments with leading '#'.
#
# Loaded via the asciidoctor -r flag from the docs Submakefile.

require "rouge"

module Rouge
  module Lexers
    class LinuxCNCINI < Rouge::Lexers::INI
      title "INI"
      desc "LinuxCNC INI dialect with # comments"

      state :root do
        # add # comments
        rule %r/#.*$/, Comment::Single

        # keep existing ; comments
        rule %r/;.*$/, Comment::Single

        mixin :base
      end
    end
  end
end
