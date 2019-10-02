#!/usr/bin/perl -w
use warnings;
use strict;
use bignum;
use Time::HiRes qw(usleep nanosleep);

use strict;
use warnings;
use Device::SerialPort qw( :PARAM :STAT 0.07 );

$SIG{INT} = sub {save_output() ; die "force exited"} ;
my $maker = shift;
my $type  = shift;
$|++;    # turn off buffering. flush writes to screen instantly
my $gantry = init_gantry();
my %data;
my %locations;
my $ft                         = init_ft();
my $keyscanner                 = init_keyscanner();
my $min_step = 0.04375/3;

my $step                       = $min_step;
# My gantry is a monoprice select mini v2.
# Per https://www.mpselectmini.com/optimal_layer
# So that motor [Z-Axis] is a 7.5°, 48 step motor as I just listed. Since
# the motor is attached to a M4 rod, which has a 0.7 mm thread pitch, then
# in one revolution makes the Z-Axis travel up or down 0.7 mm. Since it took
# 48 steps to turn that rev, each step is 0.0145833333333333333333333333333
# etc etc mm. To avoid rounding errors, you can use multiple of 3 of this
# number, which is a nice and pretty 0.04375 mm. That is a nice and handy
# number that effectively represents the layer heights that mathematically
# work the best for layer heights for this printer.
my $samples_per_step           = 3;
my $runs                       = 10;
my $max_keypress_force         = 110;
my $force_tester_bailout_force = 1000;
my $location_counter = 0;

my $last_result = 0;
for (my $run = 1; $run <= $runs; $run++) {
#    $samples_per_step=$run;
    probe_for_switch_top();
    warn "# Downstroke\n";
    while (1) {
        move_gantry_z(0 - $step);
	($last_result, $location_counter) = record_reading($run,'downstroke', $step, $location_counter);
	$location_counter++;

        if ($last_result > $max_keypress_force) {
            warn "# Downstroke bottomed out after detecting $max_keypress_force g of force";
            last;
        }
    }

    warn "# Upstroke\n";
    while (1) {
        move_gantry_z($step);
	($last_result, $location_counter) = record_reading($run,'upstroke', $step, $location_counter);
	$location_counter--;


        if (($last_result <= 0 && $location_counter < -1) || $location_counter < -8) {
            warn "# Upstroke all done\n";
            last;
        }

    }

}

save_output();
exit;

sub probe_for_switch_top {
    while (1) {
        move_gantry_z(0-$step);
        my $result = average_n_force_measurements(2);
        warn "Dropping gantry to probe for key top. Got force $result\n";
        if ($result > 1) {
            warn "We're good to go: We've homed to the top of the switch";
            last;
        }
    }
    move_gantry_z($step*2);
    $location_counter = -2;
	sleep(1);
    # Zero the force tester
    run_force_tester_cmd($ft, 0xaa, 0x01, 0x55);
	$ft->purge_rx();
    my $result = average_n_force_measurements(10);

}

sub save_output {

    mkdir('force-tests');
    mkdir('force-tests/' . $maker);
    mkdir('force-tests/' . $maker . "/" . $type);
    open(my $outfile, ">",
        "force-tests/$maker/$type/$maker-$type-step-$step-mm-" . $samples_per_step . "-samples-averaged-per-step-" . time() . ".csv");

    print $outfile "position,";
    foreach my $run (sort keys %data) {
        print $outfile "Run $run - downstroke, Run $run - downstroke actuation, Run $run - upstroke, Run $run - upstroke actuation,";
    }
    print $outfile "\n";
    foreach my $location (sort {$a <=> $b} keys %locations) {
        print $outfile $location . ",";
        foreach my $run (sort {$a <=> $b} keys %data) {
            print $outfile ($data{$run}->{downstroke}->{$location}->{force}    || '') . ",";
            print $outfile ($data{$run}->{downstroke}->{$location}->{actuated} || '') . ",";
            print $outfile ($data{$run}->{upstroke}->{$location}->{force}      || '') . ",";
            print $outfile ($data{$run}->{upstroke}->{$location}->{actuated}   || '') . ",";

        }
        print $outfile "\n";
    }

}

sub move_gantry_z {
    my $movement = shift;

    run_gantry_cmd($gantry, "G91");                # set motion to relative
    run_gantry_cmd($gantry, "G1 Z" . $movement);
    usleep(20);                                    # give it time to get there
}

sub record_reading {
    my $run      = shift;
    my $stroke   = shift;
    my $step = shift;
    my $location_counter = shift;
    $ft->purge_rx();
    my $result   = average_n_force_measurements($samples_per_step);

    if ($location_counter <= 0 && $result > 0 && $stroke eq 'downstroke') {	
	warn "Just reset our first reading with force from ".($location_counter *$step) . " to $step\n";
	$location_counter = 1;	
    }
    
    my $location = $location_counter * $step;

    my $actuated = read_keyscanner($keyscanner);

    print "Run $run - $stroke - $location mm: $result g - ";

    if ($actuated != 0) {
        print "Actuated: $actuated";
    }
    print " Delta " . ($last_result - $result)."\n";

    $data{$run}->{$stroke}->{$location}->{force}    = $result;
    $data{$run}->{$stroke}->{$location}->{actuated} = $actuated;
    $locations{$location}                           = 1;

    return ($result, $location_counter);
}

sub bail_out {
	move_gantry_z(10);
    die "We had something crazy happen. bailed.";
}


sub average_n_force_measurements {
    my $samples = shift;
    my $result  = 0;
    # usleep(10000);
    my @samples;
    for (my $i = 0; $i < ( $samples); $i++) {
  	print "+";	
        push @samples, get_next_force_measurement();
	print " ".$samples[-1]." ";

	
    }
  
    map { $result += $_ } @samples;
    $result = $result / $samples;   
    return $result;
}

my @current_force_measurement;
sub get_next_force_measurement {

    shift @current_force_measurement;
    while (1) {
    	$ft->purge_rx();
	for (my $i = 0; $i< 50; $i++) {
        my ($count_in,$bytes) = $ft->read(7);
	if ($count_in) {
			warn "Only got $count_in bytes when we expected 7" if ($count_in != 7);
            my @bytes = split(//, $bytes);
            while ($#bytes >=0) {
	 my $byte = shift @bytes;
                if (ord($byte) == 0x55 && ($#current_force_measurement >= 5)) {    # 0xaa is 170
		    my @result = (@current_force_measurement, $byte);
                    @current_force_measurement = (@bytes);
                    if (ord($result[0]) != 0xAA || ord($result[6]) != 0x55 || ((ord($result[5]) != 0x2C) && (ord($result[5]) != 0x0C))) {
                        warn "had bad result in our measurement";
                        return get_next_force_measurement();
                    }
                    my $value = extract_base256_force_value(@result);
                    return sanity_check_force_value($value);

                } elsif ($#current_force_measurement > 5) {
                    warn "bad buffer";
                    for my $b (@current_force_measurement) {
                        print ord($b) . ",";
                    }
                     return get_next_force_measurement();
                 } 

                push @current_force_measurement, $byte;
            }

        }
		   usleep(20000);
	print ".";
    }
	warn "Tried to get a measurement 50 times and failed. restarting comms with the force probe\n";
 $ft                         = init_ft();
	}
}

sub read_keyscanner {
    my $keyscanner = shift;
    my $InBytes    = 1;

    my $count_in = 0;
    my $string_in;

    ($count_in, $string_in) = $keyscanner->read($InBytes);

    #    $keyscanner->purge_rx();

    my $bytes = $string_in;
    if (!defined $bytes) {
        die "nothing from the keyscanner. is it connected and transmitting?";
    }
    if    ($bytes == 0) {return -1;}
    elsif ($bytes == 1) {return 1}
    else                {return 0}
}

sub run_gantry_cmd {
    my $gantry        = shift;
    my $output_string = shift;
    my $count_out     = $gantry->write($output_string . "\r");
    warn "write failed\n" unless ($count_out);
}

sub init_keyscanner {

    my $keyscanner_port = '/dev/serial/by-id/usb-Arduino__www.arduino.cc__0043_5573631353735170C021-if00';
    my $keyscanner      = Device::SerialPort->new($keyscanner_port, 1)
        || die "Can't open $keyscanner_port: $!";

    my $data   = $keyscanner->databits(8);
    my $baud   = $keyscanner->baudrate(57600);
    my $parity = $keyscanner->parity("none");
    $keyscanner->buffers(1, 1);
    $keyscanner->stopbits(1);
    $keyscanner->write_settings or die "no settings\n";

    return $keyscanner;

}

sub init_ft {

    my $force_tester_port = '/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0';
    my $ft                = Device::SerialPort->new($force_tester_port, 1)
        || die "Can't open $force_tester_port: $!";

    my $data   = $ft->databits(8);
    my $baud   = $ft->baudrate(57600);
    my $parity = $ft->parity("none");
$ft->read_char_time(5);
$ft->read_const_time(500);

    #$ft->buffers(7, 7);
    $ft->stopbits(1);
    $ft->handshake('none');

    $ft->write_settings or die "no settings\n";

    # Set output to kg f
    run_force_tester_cmd($ft, 0xaa, 0x04, 0x55);

    # set output to realtime
    #run_force_tester_cmd($ft,0xaa,0x03,0x55);
    sleep(1);

    # Zero the force tester
    run_force_tester_cmd($ft, 0xaa, 0x01, 0x55);
    return $ft;

}

sub run_force_tester_cmd {
    my $ft            = shift;
    my @bytes         = (@_);
    my $output_string = pack("C*", @bytes);
    my $count_out     = $ft->write($output_string);
    warn "write failed\n" unless ($count_out);
}

sub init_gantry {

    my $gantry_port = '/dev/serial/by-id/usb-Malyan_System_Malyan_3D_Printer_2060396E4752-if00';
    my $gantry      = Device::SerialPort->new($gantry_port, 1)
        || die "Can't open $gantry_port: $!";

    my $data   = $gantry->databits(8);
    my $baud   = $gantry->baudrate(19200);
    my $parity = $gantry->parity("none");
    $gantry->buffers(8, 8);
    $gantry->stopbits(1);
    $gantry->handshake('none');

    $gantry->write_settings or die "no settings\n";
    run_gantry_cmd($gantry, "G21");    # set output to mm;
    run_gantry_cmd($gantry, "G91");    # set motion to relative

    return $gantry;
}

sub extract_base256_force_value {
    my @data  = @_;
    my $value = ord($data[4]) + (256 * ord($data[3])) + (256 * 256 * ord($data[2])) + (256 * 256 * 256 * ord($data[1]));
    if (ord($data[5]) == 0x0C) {
        $value = 0 - $value;
    }
    return $value;
}

sub sanity_check_force_value {
    my $value = shift;
    if ($value > 10000) {
        warn "Got a crazy measurement. discarding\n";
        return get_next_force_measurement();
    } elsif ($value > $force_tester_bailout_force) {
        move_gantry_z(5);
        die "Got a force measurement of $value. Raising the gantry 5mm and exiting";
    } else {

        return $value;
    }
}
