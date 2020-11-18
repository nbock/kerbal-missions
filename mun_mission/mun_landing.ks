// Checks if resources in stage are depleted, so we should stage
function check_stage {
    // checks if liquid fuel is empty and stages
    until stage:resources[0]:amount > 0.1 {
        stage.
        wait until stage:ready.
    }
}


// Helper function for finding correct heading based on altitude
function find_heading {
    if altitude < 5000 {
        set steering to heading (90, 80).
    } else if altitude < 10000 {
        set steering to heading (90, 60).
    } else if altitude < 15000 {
        set steering to heading (90, 50).
    } else if altitude < 20000 {
        set steering to heading (90, 44).
    } else {
        set steering to heading (90, 38).
    }
}

// sets the orbit (non-circular) around some body
function set_orbit {
    // body to orbit around
    parameter body.
    parameter orb.

    until orb:apoapsis > body:atm:height + 5000 and orb:periapsis > body:atm:height + 5000 and orb:apoapsis < body:soiradius {
        check_stage().

        // Adde in: orb:apoapsis < body:atm:height + 5000 or
        if orb:apoapsis < body:atm:height + 5000 or orb:apoapsis > body:soiradius {
            set steering to retrograde.

            if eta:periapsis < 35 {
                lock throttle to 1.0.
            } else if eta:periapsis > 60 {
                set steering to heading (90, 0).
                lock throttle to 0.0.
            }
        } else {
            set steering to prograde.

            if eta:apoapsis < 35 or eta:apoapsis > eta:periapsis {
                lock throttle to 1.0.
            } else if eta:apoapsis > 60 {
                set steering to heading (90, 0).
                lock throttle to 0.0.
            }
        }
    }
    lock throttle to 0.0.
}

// capture burn helper function
function captureBurn {
    // body to orbit around
    parameter bod1.

    lock steering to retrograde.

    until ship:orbit:body = bod1 {
        wait 1.
    }

    until apoapsis > (bod1:atm:height + 10000) and apoapsis < 200000 {
        if eta:periapsis < 35 {
            lock throttle to 1.0.
        }
    }
    lock throttle to 0.0.
}


// circularization burn calculation adapted from Lucius_Martius' comment on this
// thread: https://www.reddit.com/r/Kos/comments/2wuo9o/what_is_the_easiest_way_to_circularize_while/
// Adapted by Nolan Bock
// Takes a body to circularize around as a parameter and returns a maneuver node
function circularize {
    // the body we will circularize around
    parameter body.

    set mu to body:mu.
    set body_radius to body:radius.

    // calculations for delta v to circularize orbit
    set r_ap to ship:apoapsis + body_radius.
    set r_pe to ship:periapsis + body_radius.
    set dv to (sqrt(mu/r_ap) - sqrt((r_pe * mu) / (r_ap * (r_pe + r_ap)/2))).
    set max_acc to ship:maxthrust/ship:mass.
    set burn_duration to dv/max_acc.

    // equalize the burn on both sides of the apoapsis
    set n_time to (eta:apoapsis - round(burn_duration) / 2).
    set nd to NODE(time:seconds + n_time, 0, 0, dv).
    return nd.
}

// Launch works for the Kerbal X with a kOS attached
// or a rocket that stages similarly (stages when fuel is empty for a stage)
function launch {
    gear off.
    lock throttle to 1.0.
    set liftoff to true.
    until ship:apoapsis > 100000 {
        if liftoff {
            stage.
            set liftoff to false.
        }
        find_heading().
        check_stage().
    }
    lock throttle to 0.0.
}

// Execute node, adapted from kOS manual on maneuver nodes
// Access from https://ksp-kos.github.io/KOS/tutorials/exenode.html
function execute_node {
    parameter node.

    set nd to node.
    add nd.

    // print out node's basic parameters - ETA and deltaV
    print "Node in: " + round(nd:eta) + ", DeltaV: " + round(nd:deltav:mag).

    set max_acc to ship:maxthrust/ship:mass.
    set burn_duration to nd:deltav:mag/max_acc.

    set np to nd:deltav. //points to node, don't care about the roll direction.
    lock steering to np.

    // now we need to wait until the burn vector and ship's facing are aligned
    wait until vang(np, ship:facing:vector) < 0.25.

    // the ship is facing the right direction, let's wait for our burn time
    print "waiting...".
    wait until nd:eta <= (burn_duration/2).
    print "waited".

    set tset to 0.
    lock throttle to tset.

    set done to False.
    set dv0 to nd:deltav.
    until done
    {
        check_stage().
        // recalculate current max_acceleration, as it changes while we burn through fuel
        set max_acc to ship:maxthrust/ship:mass.

        // throttle is 100% until there is less than 1 second of time left to burn
        // when there is less than 1 second - decrease the throttle linearly
        set tset to min(nd:deltav:mag/max_acc, 1).

        // here's the tricky part, we need to cut the throttle as soon as our nd:deltav and initial deltav start facing opposite directions
        // this check is done via checking the dot product of those 2 vectors
        if vdot(dv0, nd:deltav) < 0
        {
            print "End burn, remain dv " + round(nd:deltav:mag,1) + "m/s, vdot: " + round(vdot(dv0, nd:deltav),1).
            lock throttle to 0.
            break.
        }

        // we have very little left to burn, less then 0.1m/s
        if nd:deltav:mag < 0.1
        {
            print "Finalizing burn, remain dv " + round(nd:deltav:mag,1) + "m/s, vdot: " + round(vdot(dv0, nd:deltav),1).
            //we burn slowly until our node vector starts to drift significantly from initial vector
            //this usually means we are on point
            wait until vdot(dv0, nd:deltav) < 0.5.

            lock throttle to 0.
            print "End burn, remain dv " + round(nd:deltav:mag,1) + "m/s, vdot: " + round(vdot(dv0, nd:deltav),1).
            set done to True.
        }
    }
    unlock steering.
    lock throttle to 0.0.

    // we no longer need the maneuver node
    remove nd.
}


// Calculate Hohmann transfer logic.
// My Hohmann transfer was supplemented with Alex Ascherson's: https://github.com/AlexAscherson/Kerbal-Kos-Programming
// Lightly adapted by Nolan Bock to estimate times
function CalcHohmannTransfer {
    parameter tgtbody.

	// move origin to central body
    set positionlocal to V(0,0,0) - body:position.
    set positiontarget to tgtbody:position - body:position.

    // Hohmann transfer orbit period
    set bodyradius to body:radius.
    set altitudecurrent to bodyradius + altitude.                 // actual distance to body
    set altitudeaverage to bodyradius + (periapsis+apoapsis)/2.  // average radius (burn angle not yet known)
    set currentvelocity to ship:velocity:orbit:mag.          // actual velocity
    set averagevelocity to sqrt( currentvelocity^2 - 2*body:mu*(1/altitudeaverage - 1/altitudecurrent) ). // average velocity
    set soi to (tgtbody:soiradius).
    set transferAp to positiontarget:mag - (0.3*soi).

    // Transfer SMA
    set sma_transfer to (altitudeaverage + transferAp)/2.
    set transfertime to 2 * constant():pi * sqrt(sma_transfer^3/body:mu).

    // current target angular position
    set targetangularpostioncurrent to arctan2(positiontarget:x,positiontarget:z).
    // target angular position after transfer
    set target_sma to positiontarget:mag.                       // mun/minmus have a circular orbit
    set orbitalperiodtarget to 2 * constant():pi * sqrt(target_sma^3/body:mu).      // mun/minmus orbital period
    set sma_ship to positionlocal:mag.
    set orbitalperiodship to 2 * constant():pi * sqrt(sma_ship^3/body:mu).      // ship orbital period

    set transferangle to (transfertime/2) / orbitalperiodtarget * 360.            // mun/minmus angle for hohmann transfer
    set das to (orbitalperiodship/2) / orbitalperiodtarget * 360.           // half a ship orbit to reduce max error to half orbital period

    set at1 to targetangularpostioncurrent - das - transferangle.                // assume counterclockwise orbits

    // current ship angular position
    set shipangularpostion_current to arctan2(positionlocal:x,positionlocal:z).

    // ship angular position for maneuver
    set shipangularpostion_manuever_temp to mod(at1 + 180, 360).

    // eta to maneuver node
    set shipangularpostion_manuever to shipangularpostion_manuever_temp.
    until shipangularpostion_current > shipangularpostion_manuever {
        set shipangularpostion_manuever to shipangularpostion_manuever - 360.
    }
    set etanode to (shipangularpostion_current - shipangularpostion_manuever) / 360 * orbitalperiodship.

    // hohmann orbit properties
    set transferdv to sqrt( averagevelocity^2 - body:mu * (1/sma_transfer - 1/sma_ship ) ).
    set dv to transferdv - averagevelocity.

    set delaynode to 0.
    // setup node
    if delaynode = 0 {
      set nd to node(time:seconds + etanode, 0, 0, dv).
    } else {
      set nd to node(time:seconds + (delaynode+ etanode), 0, 0, dv).
    }

    return nd.
}

// This code was written by brekus on this link: https://www.reddit.com/r/Kos/comments/3u1l5s/how_do_you_get_your_crafts_altitude_off_the_ground/
// Adapted by Nolan Bock
// Helper function for finding the height of a ship
function find_height {
    list parts in partList.
    set lp to 0. // lowest part height
    set hp to 0. // hightest part height

    for p in partList{
        set cp to facing:vector * p:position.
    if cp < lp
        set lp to cp.
    else if cp > hp
        set hp to cp.
    }

    set height to hp - lp.
    return height.
}

// Helper function to hover to landing on the mun
function lunar_land {
    print "Beginning munar landing".
    // first, we need to bring our ship down

    set apBurn to false.
    until apBurn {
        if eta:apoapsis < 30 {
            set apBurn to true.
        }
    }

    until periapsis < 0 {
        lock steering to retrograde.

        lock throttle to 0.3.
    }
    lock throttle to 0.0.

    // get ready to land
    // get to stage 0 for lunar landing
    until stage:number = 0 {
        stage.
        wait until stage:ready.
    }
    set touchdown to false.
    set height to find_height().

    lock throttle to 0.0.
    gear on.
    lights on.
    clearscreen.
    until touchdown {
        //set st to R(0,0,0) * velocity:surface.
        lock steering to srfretrograde.

        set alt_actual to alt:radar - height.

        if ship:status = "landed" {
            lock throttle to 0.0.
            set touchdown to true.
        } else if alt_actual < 1 {
            lock throttle to max(th - 0.1, 0).
        } else if alt_actual < 10 and airspeed > 1 {
            set th to min(th + 0.1, 1).
        } else if alt_actual < 50 and airspeed > 2 {
            set th to min(th + 0.1, 1).
        } else if alt_actual < 100 and airspeed > 10 {
            set th to min(th + 0.1, 1).
        } else if alt_actual < 250 and airspeed > 20 {
            set th to min(th + 0.1, 1).
        } else if alt_actual < 500 and airspeed > 30 {
            set th to min(th + 0.1, 1).
        } else if alt_actual < 2000 and airspeed > 100 {
            set th to min(th + 0.1, 1).
        } else {
            set th to 0.0.
        }
        print "throttle: " + round(th,4) at(0,1).
        print "airspeed: " + round(airspeed,2) at(0,2).
        print "altitude: " + round(alt_actual,2) at(0,3).
        //lock steering to -st.
        lock throttle to th.
    }
    clearscreen.
    print "Mun touchdown!".
}

// main function to automate a launch to mun landing
function main {
    // launch to kerbin orbit
    launch().

    // set orbit around kerbin and circularize
    set_orbit(kerbin, orbit).
    set node to circularize(kerbin).
    execute_node(node).

    // transfer to the Mun via a Hohmann transfer
    set transfer_node to CalcHohmannTransfer(mun).
    execute_node(transfer_node).

    // capture burn around the mun
    captureBurn(mun).

    // land on the mu
    lunar_land().
}

main().
