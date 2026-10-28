# Prefer a normal TeX Live installation when available. On this workstation,
# Tectonic supplies the TeX engine and performs the reference-resolution passes.
# Keep its log and intermediate files so latexmk can track dependencies.
if (system('pdflatex --version >/dev/null 2>&1') != 0
        && system('tectonic --version >/dev/null 2>&1') == 0) {
    $pdflatex = 'internal curo6r_tectonic %S %D';
    $recorder = 0;
}

sub curo6r_tectonic {
    my ($source, $pdf) = @_;
    my $directory = File::Basename::dirname($pdf);
    my $status = system('tectonic', '--keep-logs', '--keep-intermediates',
                        '--outdir', $directory, $source);
    return $status if $status;
    return 1 unless -s $pdf;

    # Tectonic omits the banner latexmk requires and records the intermediate
    # XDV in the TeX log. Adapt only that metadata after successful PDF output.
    my $log = $pdf;
    $log =~ s/\.pdf$/.log/;
    open(my $input, '<', $log) or die "Cannot read $log: $!";
    my $text = do { local $/; <$input> };
    close($input);
    $text = "This is XeTeX, via Tectonic (latexmk adapter)\n" . $text
        unless $text =~ /^This is /;
    my $bytes = -s $pdf;
    $text =~ s/^Output written on .+\.xdv \((\d+) pages?, \d+ bytes\)\./Output written on $pdf ($1 pages, $bytes bytes)./m;
    open(my $output, '>', $log) or die "Cannot write $log: $!";
    print {$output} $text;
    close($output);
    return 0;
}
