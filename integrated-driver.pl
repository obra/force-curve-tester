#!/usr/bin/perl -w
use warnings;
use strict;
use bignum;

use strict;
use warnings;
use Device::SerialPort qw( :PARAM :STAT 0.07 );
my $maker = shift;
my $type = shift;

my $gantry = init_gantry();


  
my $ft = init_ft(); 

my $step = 0.01;
my $samples_per_step=10;
my $runs = 3;
my $force_max=180;
open (my $outfile, ">", "tester-$maker-$type-step-$step-mm-".$samples_per_step."-samples-averaged-per-step-".time().".csv");


for(my $run = 1; $run <= $runs; $run++) {

while (1) {

run_gantry_cmd($gantry, "G1 Z-0.1");

my $result = average_n_measurements($samples_per_step);

if ($result > 2) {
	warn "We're good to go: We've homed to the top of the switch";
	last;
}
}

run_gantry_cmd($gantry, "G1 Z1");
# Zero the force tester
#run_force_tester_cmd($ft,0xaa,0x01,0x55);

# Throw away 10 measurements before we start off.
my $result = average_n_measurements(10);

my @data;
my $location = -1;
warn "# Downstroke\n";
while (1) {

run_gantry_cmd($gantry, "G1 Z-".$step);

my $result = average_n_measurements($samples_per_step);
push @data, [$run,'downstroke',$location, $result];
warn "Run $run - Downstroke - $location mm: $result g\n";
$location+= $step;
if ($result > $force_max) {
	warn "# Bottomed out after detecting $force_max g of force";
	last;
}

}

warn "Upstroke\n";
while (1) {

run_gantry_cmd($gantry, "G1 Z".$step);

my $result = average_n_measurements($samples_per_step);
warn "Run - $run - Upstroke - $location mm: $result g\n";
push @data, [$run,'upstroke',$location, $result];
$location-= $step;
if ($result <=0 && $location < -1 ) {
	warn "# All done!\n";
	last;
}

}
for my $row (@data) {
print $outfile join(",",@$row)."\n";
}

}
exit;

sub average_n_measurements {
	my $samples = shift;
my $result = 0;
for(my $i = 0; $i<$samples;$i++) {
my $point = get_next_force_measurement();
$result += $point;
}
$result = $result/$samples; # 
return $result;
}

my @current_force_measurement;

sub get_next_force_measurement {

while(1){
    if (my $byte = $ft->read(1)) {
	if (ord($byte) ==  0xAA && ($#current_force_measurement >=5)) {  # 170
		#		previous packet done;	
		my @result = @current_force_measurement;
		@current_force_measurement =  ($byte);
		if ($#result == 6) {
	 	return	return_measurement(@result);
		}
	} else {
		push @current_force_measurement, $byte;
	}
		
    }
}
}




#run_gantry_cmd("G1 Z".$offset);





sub run_gantry_cmd {
	my $gantry = shift;
	my $output_string =shift;
	my       $count_out = $gantry->write($output_string."\r");
      warn "write failed\n"         unless ($count_out);
}




sub init_ft {

my $force_tester_port = '/dev/ttyUSB0';
my $ft = Device::SerialPort->new($force_tester_port, 1)
    || die "Can't open $force_tester_port: $!";

my $data = $ft->databits(8);
my $baud = $ft->baudrate(57600);
my $parity = $ft->parity("none");
$ft->buffers(7,7);
$ft->stopbits(1);
$ft->handshake('none');
#$ft->handshake("rts");


$ft->write_settings or die "no settings\n";


# Set output to kg f
run_force_tester_cmd($ft,0xaa,0x04,0x55);
# set output to realtime
#run_force_tester_cmd($ft,0xaa,0x03,0x55);
return $ft;

}

sub run_force_tester_cmd{
	my $ft = shift;
	my @bytes = (@_);
my $output_string = pack("C*", @bytes);
my       $count_out = $ft->write($output_string);
      warn "write failed\n"         unless ($count_out);
}

sub init_gantry {


my $gantry_port = '/dev/ttyACM1';
my $gantry = Device::SerialPort->new($gantry_port, 1)
    || die "Can't open $gantry_port: $!";
    

my $data = $gantry->databits(8);
my $baud = $gantry->baudrate(19200);
my $parity = $gantry->parity("none");
$gantry->buffers(8,8);
$gantry->stopbits(1);
$gantry->handshake('none');
#$gantry->handshake("rts");

$gantry->write_settings or die "no settings\n";
run_gantry_cmd($gantry,"G21") ; # set output to mm;
run_gantry_cmd($gantry,"G91") ; # set motion to relative

return $gantry;
}


sub return_measurement {
	my @data = (@_);
	if (ord($data[0]) == 0xAA &&  ord($data[6]) == 0x55) {
		if (ord($data[5]) == 0x2C) {
		} elsif (ord($data[5]) == 0x0C) {
		} else { warn "neither pos nor neg: ".ord($data[5]);}
		my $value = ord($data[4]) + (256 * ord($data[3])) + (256 * 256 * ord($data[2])) + (256 * 256 * 256 * ord($data[1]));
		if (ord($data[5]) == 0x0C) {
			$value = 0-$value;
		}
		#warn "Force measured: ".$value."\n";
		return $value;
	}	
	else {
		die "had bad data in our measurement";
	}
}
