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

    set warped to false.

    until apoapsis > body:atm:height + 5000 and periapsis > body:atm:height + 5000 and apoapsis < body:soiradius {
        check_stage().

        // Took out: orb:apoapsis < body:atm:height + 5000 or
        if orb:apoapsis > body:soiradius {
            set steering to retrograde.
            if not warped {
                warpto(time:seconds + eta:transition).
                wait until eta:transition < 10.
                warpto(time:seconds + (eta:periapsis - 60)).
                set warped to true.
            }

            if eta:periapsis < 35 {
                lock throttle to 1.0.
            } else if eta:periapsis > 60 {
                set steering to heading (90, 0).
                lock throttle to 0.0.
            }
        } else {
            set steering to prograde.
            if not warped {
                warpto(time:seconds + (eta:apoapsis - 60)).
                set warped to true.
            }
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
// stage:resources[0] is 0, all liquid burns.
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
    warpto(time:seconds + nd:eta - (burn_duration/2) - 20).
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

// Transfer orbit helper function transfers from current to tgtbody
// This currently does nothing
function transfer_orbit {
    parameter tgtbody.
    parameter transfer_node.

    set done to False.
    set delaynode to 0.

    until done {
        CalcHohmannTransfer(tgtbody).
        print "Checking result".
        print tgtbody + "this is targetbody".

        if encounter = "none" {
            print "T+" + round(missiontime) + " WARNING! No encounter found.".

            set done to True.
            return false.
            // Need some logic here to wait for longer or warp till maybe moon gets out of the way.
        } else {
            if encounter:body:name = tgtbody:name {
                set done to True.
            }
        }
    }
}


function eta_true_anom {
    declare local parameter tgt_lng.
    // convert the positon from reference to deg from PE (which is the true anomaly)
    local ship_ref to mod(obt:lan+obt:argumentofperiapsis+obt:trueanomaly,360).
    // s_ref = lan + arg + referenc

    local node_true_anom to (mod (720+ tgt_lng - (obt:lan + obt:argumentofperiapsis),360)).

    print "Node anomaly   : " + round(node_true_anom,2).
    local node_eta to 0.
    local ecc to OBT:ECCENTRICITY.
    if ecc < 0.001 {
        set node_eta to SHIP:OBT:PERIOD * ((mod(tgt_lng - ship_ref + 360,360))) / 360.

    } else {
        local eccentric_anomaly to  arccos((ecc + cos(node_true_anom)) / (1 + ecc * cos(node_true_anom))).
        local mean_anom to (eccentric_anomaly - ((180 / (constant():pi)) * (ecc * sin(eccentric_anomaly)))).

        // time from periapsis to point
        local time_2_anom to  SHIP:OBT:PERIOD * mean_anom /360.

        local my_time_in_orbit to ((OBT:MEANANOMALYATEPOCH)*OBT:PERIOD /360).
        set node_eta to mod(OBT:PERIOD + time_2_anom - my_time_in_orbit,OBT:PERIOD) .

    }

    return node_eta.
}


// Inclination logic from Alex Ascherson's: https://github.com/AlexAscherson/Kerbal-Kos-Programming
// Lightly adapted by Nolan Bock to estimate times
function set_inc_lan {
    DECLARE PARAMETER incl_t.
    DECLARE PARAMETER lan_t.
    local incl_i to SHIP:OBT:INCLINATION.
    local lan_i to SHIP:OBT:LAN.

    // setup the vectors to highest latitude; Transform spherical to cubic coordinates.
    local Va to V(sin(incl_i)*cos(lan_i+90),sin(incl_i)*sin(lan_i+90),cos(incl_i)).
    local Vb to V(sin(incl_t)*cos(lan_t+90),sin(incl_t)*sin(lan_t+90),cos(incl_t)).
    // important to use the reverse order
    local Vc to VCRS(Vb,Va).

    local dv_factor to 1.
    //compute burn_point and set to the range of [0,360]
    local node_lng to mod(arctan2(Vc:Y,Vc:X)+360,360).
    local ship_ref to mod(obt:lan+obt:argumentofperiapsis+obt:trueanomaly,360).

    local ship_2_node to mod((720 + node_lng - ship_ref),360).
    if ship_2_node > 180 {  // Might need to check if the nextnode ri to target is better in ###as it gets it wrong sometimes.
        print "Switching to DN".
        set dv_factor to -1.
        set node_lng to mod(node_lng + 180,360).
    }

    local node_true_anom to 360- mod(720 + (obt:lan + obt:argumentofperiapsis) - node_lng , 360 ).
    local ecc to OBT:ECCENTRICITY.
    local my_radius to OBT:SEMIMAJORAXIS * (( 1 - ecc^2)/ (1 + ecc*cos(node_true_anom)) ).
    local my_speed1 to sqrt(SHIP:BODY:MU * ((2/my_radius) - (1/OBT:SEMIMAJORAXIS)) ).
    local node_eta to eta_true_anom(node_lng).
    local my_speed to VELOCITYAT(SHIP, time+node_eta):ORBIT:MAG.
    local d_inc to arccos (vdot(Vb,Va) ).
    local dvtgt to dv_factor* (2 * (my_speed) * SIN(d_inc/2)).

    // Create a blank node
    local inc_node to NODE(node_eta, 0, 0, 0).
    // we need to split our dV to normal and prograde
    set inc_node:NORMAL to dvtgt * cos(d_inc/2).
    // always burn retrograde
    set inc_node:PROGRADE to 0 - abs(dvtgt * sin(d_inc/2)).
    set inc_node:ETA to node_eta.

    return inc_node.
}

function setInclination {
    parameter tgtbody.

    set target to tgtbody.

    set ri to abs(obt:inclination - (tgtbody:obt:inclination+0.01)).
    // Align if necessary
    //if ((ship:orbit:inclination + tgtbody:orbit:inclination-0.2)/2) > (tgtbody:orbit:inclination +0.2) or ((ship:orbit:inclination + tgtbody:orbit:inclination)/2) < (tgtbody:orbit:inclination -0.2)
    if ri > 0.1 {
        // print "Matching Inclination".
        set inc_node to set_inc_lan(tgtbody:orbit:inclination, tgtbody:orbit:LAN).
        return inc_node.
    }
    return "None".
}

// Calculate Hohmann transfer logic.
// My Hohmann transfer was supplemented with Alex Ascherson's: https://github.com/AlexAscherson/Kerbal-Kos-Programming
// Lightly adapted by Nolan Bock to estimate times
function CalcHohmannTransfer {
    parameter tgtbody.

    set target to tgtbody.

    set done to False.
    set delaynode to 0.

    //parameter tgtbody.
	// move origin to central body (i.e. Kebodyradiusin)
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
      set nd to node(time:seconds + (delaynode + etanode), 0, 0, dv).
    }

    add nd.
    set ticker to 0.
    set original to dv.
    until nd:orbit:hasnextpatch or ticker > 1000 {
        remove nd.
        set dv to (dv + 0.1).
        set ticker to ticker + 1.
        set nd to node(time:seconds + etanode, 0, 0, dv).
        add nd.
    }

    set dv to original.
    set ticker to 0.
    until nd:orbit:hasnextpatch or ticker > 1000 {
        remove nd.
        set dv to (dv - 0.1).
        set ticker to ticker + 1.
        set nd to node(time:seconds + etanode, 0, 0, dv).
        add nd.
    }

    if ticker > 1000 {
        remove nd.
        set dv to original.
        set nd to node(time:seconds + etanode, 0, 0, dv).
    }

    remove nd.
    return nd.
}



function CalcHohmannOrbit {
    parameter tgtbody.
    // the current orbit
    parameter currOrbit.

    set target to tgtbody.

    set done to False.
    set delaynode to 0.

    //parameter tgtbody.
	// move origin to central body (i.e. Kebodyradiusin)
    set positionlocal to V(0,0,0) - currOrbit:body:position.
    set positiontarget to tgtbody:position - currOrbit:body:position.

    // Hohmann transfer orbit period
    set bodyradius to currOrbit:body:radius.
    set altitudecurrent to bodyradius + altitude.                 // actual distance to body
    set altitudeaverage to bodyradius + (currOrbit:periapsis + currOrbit:apoapsis)/2.  // average radius (burn angle not yet known)
    set currentvelocity to currOrbit:velocity:orbit:mag.          // actual velocity
    set averagevelocity to sqrt( currentvelocity^2 - 2 * currOrbit:body:mu * (1/altitudeaverage - 1/altitudecurrent) ). // average velocity
    set soi to (tgtbody:soiradius).
    set transferAp to positiontarget:mag - (0.3*soi).

    // Transfer SMA
    set sma_transfer to (altitudeaverage + transferAp)/2.
    set transfertime to 2 * constant():pi * sqrt(sma_transfer^3/currOrbit:body:mu).

    // current target angular position
    set targetangularpostioncurrent to arctan2(positiontarget:x,positiontarget:z).
    // target angular position after transfer
    set target_sma to positiontarget:mag.                       // mun/minmus have a circular orbit
    set orbitalperiodtarget to 2 * constant():pi * sqrt(target_sma^3/currOrbit:body:mu).      // mun/minmus orbital period
    set sma_ship to positionlocal:mag.
    set orbitalperiodship to 2 * constant():pi * sqrt(sma_ship^3/currOrbit:body:mu).      // ship orbital period

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
    set transferdv to sqrt( averagevelocity^2 - currOrbit:body:mu * (1/sma_transfer - 1/sma_ship ) ).
    set dv to transferdv - averagevelocity.

    set delaynode to 0.
    // setup node
    if delaynode = 0 {
      set nd to node(time:seconds + etanode, 0, 0, dv).
    } else {
      set nd to node(time:seconds + (delaynode+ etanode), 0, 0, dv).
    }

    set speedAfterBurn to currOrbit:velocity:orbit:mag + dv.
    set currPos to v(0,0,0) - currOrbit:position.
    set targetPos to v(0,0,0) - tgtbody:position.
    set distanceBetweenOrbits to (currPos - targetPos):mag.
    set travelTime to distanceBetweenOrbits / speedAfterBurn.
    set travelTimeMinutes to travelTime / 60.
    set travelTimeHours to travelTimeMinutes / 60.
    set ret to list(nd, dv, currOrbit:velocity:orbit:mag + dv, distanceBetweenOrbits, travelTime, travelTimeMinutes, travelTimeHours).
    return ret.
}

function find_path {
    set moons to orbit:body:orbitingchildren.

    if moons:empty {
        return "None".
    }

    set inc to setInclination(moons[0]).
    if ri > 0.1 {
        add inc.
    }
    set circ to CalcHohmannTransfer(moons[0]).
    add circ.
    set next to circ:orbit:nextpatcheta.
    set minTime to next.
    set minMoon to moons[0].
    if ri > 0.1 {
        remove inc.
    }
    remove circ.

    // do a while moons > 0

    set i to 0.
    set minIndex to 0.
    set order to list().
    set i to 0.
    until moons:length < 1 {
        for moon in moons {
            set minMoon to moons[0].

            set inc to setInclination(moon).
            if ri > 0.1 {
                add inc.
            }
            set circ to CalcHohmannTransfer(moon).
            add circ.
            set next to circ:orbit:nextpatcheta.

            if next < minTime {
                set minTime to next.
                set minIndex to i.
                set minMoon to moon.
            }
            if ri > 0.1 {
                remove inc.
            }
            remove circ.
            set i to i + 1.
        }
        moons:remove(minIndex).
        order:add(minMoon).
    }

    return order.
}


// escapes from the current orbit
// intended use is to escape moon orbits and move to planet
function escape {
    wait until eta:apoapsis < 60.

    until ship:apoapsis > ship:orbit:body:soiradius {
        lock steering to prograde.
        lock throttle to 1.0.
    }
    lock throttle to 0.0.
    wait 1.
    warpto(time:seconds + ship:orbit:nextpatcheta).
}

// lowers orbit to target periapsis
function lowerOrbit {
    parameter tgt.

    until periapsis < tgt {
        check_stage().
        lock steering to retrograde.
        wait 1.
        lock throttle to 0.1.
    }
    lock throttle to 0.0.

    until periapsis > orbit:body:atm:height {
        check_stage().
        lock steering to prograde.
        lock sterring to 1.0.
    }
    lock throttle to 0.0.

    wait 1.
    warpto(time:seconds + eta:periapsis - 120).
    until apoapsis < tgt or periapsis < orbit:body:atm:height + 30000 {
        check_stage().
        lock steering to retrograde.
        wait 1.
        lock throttle to 0.3.
    }
    lock throttle to 0.0.

    until apoapsis > orbit:body:atm:height {
        check_stage().
        lock steering to prograde.
        lock sterring to 1.0.
    }

    lock throttle to 0.0.
}


function visit_moon {
    parameter bd.

    print "Moving to " + bd.

    warpto(time:seconds + eta:periapsis - 30).
    wait until eta:periapsis < 20.

    set inc to setInclination(bd).
    execute_node(inc).

    // calc hohmann should have a slight adjustment
    set transfer to CalcHohmannTransfer(bd).
    add transfer.
    until transfer:orbit:nextpatch:body = bd {
        remove transfer.
        warpto(time:seconds + eta:periapsis - 120).
        set transfer to CalcHohmannTransfer(bd).
        add transfer.
    }
    remove transfer.
    execute_node(transfer).
}

function verify_orbit {
    parameter bd.

    if orbit:body = bd {
        return True.
    }
    return False.
}

// capture burn helper function
function captureBurn {
    // body to orbit around
    parameter bod1.

    until periapsis > orbit:body:atm:height + 50000 {
        lock steering to prograde.
        lock throttle to 1.0.
    }
    lock throttle to 0.0.

    lock steering to retrograde.

    until ship:orbit:body = bod1 {
        wait 1.
    }

    until apoapsis > (bod1:atm:height + 10000) and apoapsis < 200000 {
        check_stage().
        if eta:periapsis < 35 {
            lock throttle to 1.0.
        }
    }
    lock throttle to 0.0.
}


set visits to find_path().

clearscreen.
print "THE PLAN".
print " ".
print " ".
print "Orbitting: " + ship:orbit:body.
print " ".
print " ".
set i to 0.
for moon in visits {
    print "Visit " + i + ": " + moon.
    set i to i + 1.
}
print " ".
print " ".

set i to 0.
until i >= visits:length {
    print "Starting move to " + visits[i].
    visit_moon(visits[i]).

    // need some method to wait until I get to an encounter.
    wait until orbit:body = visits[i].
    print "new orbit".

    captureBurn(visits[i]).

    // know escape will release us towards orbiting body since we do it at the apoapsis
    escape().
    lowerOrbit(150000).
    set circ to circularize(kerbin).
    execute_node(circ).

    set i to i + 1.
    clearscreen.
}
