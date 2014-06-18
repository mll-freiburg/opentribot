#!/usr/bin/perl
#
# clean the clicked points from the extrinsic (i.e. last) image in a 
# cpoints.txt file
#
$|=1;

%points=();

$num=0;

open(IN,$ARGV[0]);
#780 580 13 9
chop($header=<IN>);
($width,$height,$numimgs,$numpoints)=split / /,$header;
while ($in=<IN>)
{
	chop($in);
	if ($in eq "")
	{
		$num++;
		$active=1;
	} else
	{
		if ($active == 1)
		{
			$points{$num}.=$in."\n";
		}
	}
}
close(IN);

@lines=split /\n/,$points{$numimgs};
$points{$numimgs}="";
foreach $line (@lines)
{
	($px,$py,$mx,$my) = split / /,$line;
	$mx = -1;
	$my = -1;
	$points{$numimgs}.="$px $py $mx $my\n";
}

print $header."\n";
for ($i=1;$i<=$numimgs;$i++)
{
	print "\n";
	print "$points{$i}";
}
print "\n";
