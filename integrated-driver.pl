#!/usr/bin/perl -w
use warnings;
use strict;
use bignum;
use Time::HiRes qw(usleep nanosleep);

use strict;
use warnings;
use Device::SerialPort qw( :PARAM :STAT 0.07 );
my $maker = shift;
my $type  = shift;
$|++;    # turn off buffering. flush writes to screen instantly
my $gantry = init_gantry();
my %data;
my %locations;
my $ft                         = init_ft();
my $keyscanner                 = init_keyscanner();
my $step                       = 0.01;
my $samples_per_step           = 1;
my $runs                       = 4;
my $max_keypress_force         = 180;
my $force_tester_bailout_force = 1000;
my $location                   = 0;

for (my $run = 1; $run <= $runs; $run++) {
    probe_for_switch_top();
    warn "# Downstroke\n";
    while (1) {
        move_gantry_z(0 - $step);
        my $result = record_reading('downstroke', $location);
        $location += $step;

        if ($result > $max_keypress_force) {
            warn "# Bottomed out after detecting $max_keypress_force g of force";
            last;
        }
    }

    warn "Upstroke\n";
    while (1) {
        move_gantry_z($step);
        my $result = record_reading('upstroke', $location);
        $location -= $step;
        if (($result <= 0 && $location < -0.1) || $location < -1) {
            warn "# All done!\n";
            last;
        }

    }

}

save_output();
exit;

sub probe_for_switch_top {
    while (1) {
        move_gantry_z(-0.01);
        my $result = average_n_force_measurements(1);
        warn "Dropping gantry to probe for key top. Got force $result\n";
        if ($result > 1) {
            warn "We're good to go: We've homed to the top of the switch";
            last;
        }
    }

    run_gantry_cmd($gantry, "G1 Z0.1");
    $location = -0.1;

    # Throw away 2 measurements before we start off.
    my $result = average_n_force_measurements(2);

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
    my $location = shift;
    my $result   = average_n_force_measurements($samples_per_step);
    my $actuated = read_keyscanner($keyscanner);
    print "Run - $run - $stroke - $location mm: $result g - ";
    if ($actuated != 0) {
        print "Actuated: $actuated";
    }
    print "\n";

    $data{$run}->{$stroke}->{$location}->{force}    = $result;
    $data{$run}->{$stroke}->{$location}->{actuated} = $actuated;
    $locations{$location}                           = 1;

    return $result;
}

sub bail_out {
    run_gantry_cmd($gantry, "G1 Z10");
    die "We had something crazy happen. bailed.";
}

my @current_force_measurement;

sub average_n_force_measurements {
    my $samples = shift;
    my $result  = 0;
    $ft->purge_rx();
    @current_force_measurement = ();
    for (my $i = 0; $i < $samples; $i++) {
        my $point = get_next_force_measurement();
        print $point;
        $result += $point;
    }
    $result = $result / $samples;    #
    return $result;
}

sub get_next_force_measurement {

    while (1) {
        if (my $bytes = $ft->read(8)) {
            print ".";
            my @bytes = split(//, $bytes);
            while (my $byte = shift @bytes) {
                if (ord($byte) == 0xAA && ($#current_force_measurement >= 6)) {    # 170
                    my @result = @current_force_measurement;
                    @current_force_measurement = ($byte, @bytes);
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
                }
                push @current_force_measurement, $byte;
            }

        }
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
