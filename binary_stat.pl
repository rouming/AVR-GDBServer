#!/usr/bin/perl -w

use strict;
use warnings;

my $bin = $ARGV[0] || die "No binary specified!";
my $rom_sz = $ARGV[1] || die "ROM size is specified!";
my $ram_sz = $ARGV[2] || die "RAM size is specified!";

die "Can't open binary for read!" unless -r $bin;

my $avr_objdump = `avr-objdump -h $bin`;

my $program_bytes_cnt = 0;
my $data_bytes_cnt = 0;

my @program_sects = ('.text', '.data', '.bootloader', '.nrww');
my @data_sects = ('.data', '.bss', '.noinit');

my %sects;
$sects{$_} = \$program_bytes_cnt for @program_sects;
$sects{$_} = \$data_bytes_cnt for @data_sects;

for my $dump_line (split/\n/, $avr_objdump) {
    my @dump_cols = split/\s+/, $dump_line;
    if (defined $dump_cols[2] &&
        exists $sects{$dump_cols[2]}) {
        ${$sects{$dump_cols[2]}} += hex($dump_cols[3]);
    }
}

printf "\n";
printf "Program:\t%u bytes (%.1f%% full)\n",
    $program_bytes_cnt,
    $program_bytes_cnt / $rom_sz * 100;
printf "(%s)\n",
    join ", ", @program_sects;
printf "\n";
printf "   Data:\t%u bytes (%.1f%% full)\n",
    $data_bytes_cnt,
    $data_bytes_cnt / $ram_sz * 100;
printf "(%s)\n",
    join ", ", @data_sects;
printf "\n";
