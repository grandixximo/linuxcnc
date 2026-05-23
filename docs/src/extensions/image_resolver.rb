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

    def process(document)
      # For HTML output the relative target is correct as-is — asciidoctor
      # writes it straight into <img src=...> and a sibling copy of the
      # image tree makes it resolve at deploy time. Only PDF embedding
      # needs absolute paths so prawn-svg / image reading can find the
      # file at convert time.
      return document unless document.backend == 'pdf'
      document.find_by(context: :image)        { |n| rewrite n }
      document.find_by(context: :inline_image) { |n| rewrite n }
      document
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

      candidate = File.expand_path(target, base_dir)
      candidate = resolve_extension(candidate) unless File.file?(candidate)
      return unless candidate && File.file?(candidate)

      node.set_attr('target', candidate)
      apply_default_width(node)
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
