#!/usr/bin/env python3
"""
PX4 Pre-Planned Mission Control - FIXED VERSION
Executes a perfect 10-second square pattern mission at 50Hz
- Forward 2s → Right 2s → Backward 2s → Left 2s → Forward 2s
- Maintains constant altitude
- Then asks to repeat, RTL, or land
"""

import time
import sys
import asyncio


# MISSION PLANNER - GENERATES 50Hz CONTROL SEQUENCES

class MissionPlanner:
    """
    Generates pre-planned control sequences at 50Hz (0.02s intervals)
    """
    def __init__(self):
        self.control_rate = 50  # Hz
        self.time_step = 1.0 / self.control_rate  # 0.02 seconds
        
        # IMPROVED MOVEMENT SPEEDS - Much faster
        self.forward_speed = 0.5   # m/s (increased from 0.2)
        self.right_speed = 0.5     # m/s (increased from 0.2)
        self.yaw_speed = 0.0       # deg/s (no rotation during mission)
        
        # ALTITUDE HOLD - Use zero vertical velocity instead of thrust
        self.vertical_speed = 0.0  # m/s (0 = hold altitude)
        
    def generate_mission(self):
        """
        Generate 10-second mission: Forward → Right → Backward → Left → Forward
        Returns list of control values, one per 0.02s (500 total entries)
        """
        mission = []
        
        # Each segment is 2 seconds = 100 control points at 50Hz
        segment_points = int(2.0 * self.control_rate)
        
        # Segment 1: FORWARD (0-2s) - 100 points
        print("Planning: Forward 2 seconds...")
        for i in range(segment_points):
            mission.append({
                'forward_m_s': self.forward_speed,   # Forward
                'right_m_s': 0.0,
                'down_m_s': self.vertical_speed,     # Hold altitude
                'yaw_deg_s': self.yaw_speed,
                'time': i * self.time_step,
                'phase': 'FORWARD'
            })
        
        # Segment 2: RIGHT (2-4s) - 100 points
        print("Planning: Right 2 seconds...")
        for i in range(segment_points):
            mission.append({
                'forward_m_s': 0.0,
                'right_m_s': self.right_speed,      # Right
                'down_m_s': self.vertical_speed,
                'yaw_deg_s': self.yaw_speed,
                'time': (segment_points + i) * self.time_step,
                'phase': 'RIGHT'
            })
        
        # Segment 3: BACKWARD (4-6s) - 100 points
        print("Planning: Backward 2 seconds...")
        for i in range(segment_points):
            mission.append({
                'forward_m_s': -self.forward_speed,  # Backward
                'right_m_s': 0.0,
                'down_m_s': self.vertical_speed,
                'yaw_deg_s': self.yaw_speed,
                'time': (2 * segment_points + i) * self.time_step,
                'phase': 'BACKWARD'
            })
        
        # Segment 4: LEFT (6-8s) - 100 points
        print("Planning: Left 2 seconds...")
        for i in range(segment_points):
            mission.append({
                'forward_m_s': 0.0,
                'right_m_s': -self.right_speed,     # Left
                'down_m_s': self.vertical_speed,
                'yaw_deg_s': self.yaw_speed,
                'time': (3 * segment_points + i) * self.time_step,
                'phase': 'LEFT'
            })
        
        # Segment 5: FORWARD AGAIN (8-10s) - 100 points
        print("Planning: Forward again 2 seconds...")
        for i in range(segment_points):
            mission.append({
                'forward_m_s': self.forward_speed,   # Forward
                'right_m_s': 0.0,
                'down_m_s': self.vertical_speed,
                'yaw_deg_s': self.yaw_speed,
                'time': (4 * segment_points + i) * self.time_step,
                'phase': 'FORWARD2'
            })
        
        print(f"\n✓ Mission planned: {len(mission)} control points (10 seconds at 50Hz)")
        return mission


# MAIN FLIGHT CONTROL WITH PRE-PLANNED MISSION

async def execute_planned_mission(connection_string, target_height=1.2):
    """
    Execute pre-planned mission with 50Hz control rate
    """
    from mavsdk import System
    from mavsdk.offboard import OffboardError, VelocityBodyYawspeed
    
    drone = System()
    
    print("=" * 70)
    print("PX4 PRE-PLANNED MISSION CONTROL")
    print("=" * 70)
    print(f"Target Takeoff Height: {target_height}m")
    print("Mission: Square Pattern (Forward→Right→Backward→Left→Forward)")
    print("=" * 70)
    
    print(f"\n[1/7] Connecting to PX4 at {connection_string}...")
    await drone.connect(system_address=connection_string)
    
    # Wait for connection
    print("[2/7] Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(" Drone connected!")
            break
    
    # Wait for position estimate
    print("[3/7] Waiting for position estimate...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print(" Position estimation ready")
            break
    
    # Arm the drone
    print("\n[4/7] Arming...")
    await drone.action.arm()
    print(" Armed")
    
    # Set takeoff altitude
    print(f"[5/7] Setting takeoff altitude to {target_height}m...")
    await drone.action.set_takeoff_altitude(target_height)
    
    # Takeoff
    print("[6/7] Taking off...")
    await drone.action.takeoff()
    
    # Monitor takeoff
    print(f"Monitoring takeoff to {target_height}m...")
    async for position in drone.telemetry.position():
        altitude = position.relative_altitude_m
        print(f"   Altitude: {altitude:.2f}m / {target_height}m", end='\r')
        
        if altitude >= target_height * 0.95:
            print(f"\n Reached target altitude: {altitude:.2f}m")
            break
        
        await asyncio.sleep(0.1)
    
    # Stabilize
    print("\nStabilizing for 3 seconds...")
    await asyncio.sleep(3.0)
    
    # GENERATE MISSION PLAN
    print("\n[7/7] Generating mission plan...")
    
    planner = MissionPlanner()
    mission_plan = planner.generate_mission()
    
    # START OFFBOARD MODE
    print("\nStarting OFFBOARD mode...")
    
    # Set initial setpoint - HOLD POSITION
    await drone.offboard.set_velocity_body(
        VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0)
    ) 
    
    try:
        await drone.offboard.start()
        print(" Offboard mode ACTIVE")
    except OffboardError as e:
        print(f" Failed to start offboard mode: {e}")
        print("  Make sure PX4 parameters are set:")
        print("  - COM_RCL_EXCEPT = 4")
        print("  - COM_OF_LOSS_T = 1.0")
        await drone.action.land()
        return False
    
    # Hold position for 1 second before starting mission
    print("\nHolding position for 1 second before mission start...")
    for _ in range(50):  # 1 second at 50Hz
        await drone.offboard.set_velocity_body(
            VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0)
        )
        await asyncio.sleep(0.02)
    
    print("\n" + "=" * 70)
    print("EXECUTING MISSION - PRECISE TIMING")
    print("=" * 70)
    
    # EXECUTE MISSION AT 50Hz WITH PRECISE TIMING
    
    mission_start_time = time.time()
    last_phase = None
    
    try:
        for idx, control_point in enumerate(mission_plan):
            # Calculate exact target time for this control point
            target_time = mission_start_time + control_point['time']
            
            # Send control command
            await drone.offboard.set_velocity_body(
                VelocityBodyYawspeed(
                    forward_m_s=control_point['forward_m_s'],
                    right_m_s=control_point['right_m_s'],
                    down_m_s=control_point['down_m_s'],
                    yawspeed_deg_s=control_point['yaw_deg_s']
                )
            )
            
            # Display status when phase changes
            if control_point['phase'] != last_phase:
                # Get current altitude
                current_altitude = target_height
                try:
                    async for position in drone.telemetry.position():
                        current_altitude = position.relative_altitude_m
                        break
                except:
                    pass
                
                elapsed = time.time() - mission_start_time
                print(f"\n[{elapsed:.1f}s] Starting phase: {control_point['phase']} | Altitude: {current_altitude:.2f}m")
                last_phase = control_point['phase']
            
            # Display periodic updates (every 0.5s)
            if idx % 25 == 0:
                elapsed = time.time() - mission_start_time
                progress = (idx / len(mission_plan)) * 100
                print(f"   Time: {elapsed:.1f}s | Phase: {control_point['phase']:8s} | "
                      f"Fwd: {control_point['forward_m_s']:+.1f} | "
                      f"Right: {control_point['right_m_s']:+.1f} | "
                      f"Progress: {progress:.0f}%", end='\r')
            
            # Wait until exact target time (precise timing)
            current_time = time.time()
            sleep_time = target_time - current_time
            
            if sleep_time > 0:
                await asyncio.sleep(sleep_time)
            # If we're running behind, skip the sleep but continue
    
    except KeyboardInterrupt:
        print("\n\n Mission interrupted by user")
        await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
        await asyncio.sleep(0.5)
        await drone.action.return_to_launch()
        return False
    except Exception as e:
        print(f"\n\n Error during mission: {e}")
        await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
        await asyncio.sleep(0.5)
        await drone.action.return_to_launch()
        return False
    
    mission_duration = time.time() - mission_start_time
    print(f"\n\n Mission completed in {mission_duration:.2f} seconds")
    
    # Return to hover - CRITICAL for stopping drift
    print("\nReturning to hover position (stopping all movement)...")
    for _ in range(25):  # Hold for 0.5 seconds
        await drone.offboard.set_velocity_body(
            VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0)
        )
        await asyncio.sleep(0.02)
    
    return True


async def mission_loop(connection_string, target_height):
    """
    Main mission loop - repeat, RTL, or land
    """
    from mavsdk import System
    
    drone = System()
    await drone.connect(system_address=connection_string)
    
    # Wait for connection
    async for state in drone.core.connection_state():
        if state.is_connected:
            break
    
    mission_count = 0
    
    while True:
        mission_count += 1
        print("\n" + "=" * 70)
        print(f"MISSION #{mission_count}")
        print("=" * 70)
        
        # Execute mission
        success = await execute_planned_mission(connection_string, target_height)
        
        if not success:
            print("\n Mission failed or interrupted")
            break
        
        # Ask user what to do next
        print("\n" + "=" * 70)
        print("MISSION COMPLETE - WHAT NEXT?")
        print("=" * 70)
        print("Options:")
        print("  1. Repeat mission")
        print("  2. Return to Launch (RTL)")
        print("  3. Land here")
        
        choice = input("\nSelect (1-3) [3]: ").strip() or "3"
        
        if choice == '1':
            print("\n Repeating mission...")
            continue
        elif choice == '2':
            print("\n Returning to launch...")
            # Stop offboard mode
            try:
                await drone.offboard.stop()
                print(" Offboard stopped")
            except Exception as e:
                print(f" Error stopping offboard: {e}")
            
            # RTL
            print("Executing Return to Launch...")
            await drone.action.return_to_launch()
            
            # Monitor RTL
            print("Returning to home...")
            await asyncio.sleep(5)  # Wait for RTL to complete
            break
        else:
            print("\n Landing at current position...")
            await drone.action.land()
            
            await asyncio.sleep(5) 
            break
    
    # LANDING SEQUENCE (if not RTL)
    
    if choice != '2':
        print("\n" + "=" * 70)
        print("LANDING SEQUENCE")
        print("=" * 70)
        
        # Stop offboard mode
        print("\n[Landing] Stopping offboard mode...")
        try:
            await drone.offboard.stop()
            print(" Offboard stopped")
        except Exception as e:
            print(f" Error stopping offboard: {e}")
        
        # Land
        print("[Landing] Initiating landing...")
        await drone.action.land()
        
        # Monitor landing
        print("[Landing] Descending...")
        async for position in drone.telemetry.position():
            altitude = position.relative_altitude_m
            print(f"   Altitude: {altitude:.2f}m", end='\r')
            
            if altitude < 0.1:
                print("\n Landed successfully")
                break
            
            await asyncio.sleep(0.5)
    
    print("\n" + "=" * 70)
    print(f"ALL MISSIONS COMPLETE - {mission_count} mission(s) executed")
    print("=" * 70)


# COM PORT DETECTION

def detect_com_ports():
    try:
        import serial.tools.list_ports
        return list(serial.tools.list_ports.comports())
    except ImportError:
        return []

def get_connection_string():
    print("\n" + "=" * 70)
    print("CONNECTION SETUP")
    print("=" * 70)
    
    print("\nConnection type:")
    print("1. USB Serial (COM port)")
    print("2. UDP - Simulator")
    print("3. TCP - Network")
    
    conn_type = input("\nSelect (1-3) [1]: ").strip() or "1"
    
    if conn_type == "1":
        ports = detect_com_ports()
        
        if ports:
            print("\nAvailable COM ports:")
            for i, port in enumerate(ports, 1):
                print(f"  {i}. {port.device} - {port.description}")
            
            port_choice = input(f"\nSelect port (1-{len(ports)}) or enter manually: ").strip()
            
            if port_choice.isdigit() and 1 <= int(port_choice) <= len(ports):
                com_port = ports[int(port_choice)-1].device
            else:
                com_port = port_choice.upper() if port_choice.upper().startswith("COM") else f"COM{port_choice}"
        else:
            com_port = input("\nEnter COM port (e.g., COM3): ").strip()
            com_port = com_port.upper() if com_port.upper().startswith("COM") else f"COM{com_port}"
        
        print(f"\n Using: {com_port}")
        return f"serial:///{com_port}:57600"
        
    elif conn_type == "2":
        port = input("Enter port [14550]: ").strip() or "14550"
        return f"udp://:{port}"
        
    elif conn_type == "3":
        ip = input("\nEnter IP [127.0.0.1]: ").strip() or "127.0.0.1"
        port = input("Enter port [5760]: ").strip() or "5760"
        return f"tcp:{ip}:{port}"
    
    return "serial:///COM14:57600"


# MAIN
if __name__ == "__main__":
    print("\n" + "=" * 70)
    print("PX4 PRE-PLANNED MISSION CONTROL - FIXED VERSION")
    print("Square Pattern Flight - 50Hz Control Rate")
    print("=" * 70)
    
    print("\n DEPENDENCIES:")
    print("   pip install mavsdk")
    print("\n PX4 PARAMETERS:")
    print("   - COM_RCL_EXCEPT = 4")
    print("   - COM_OF_LOSS_T = 1.0")
    print("\n MISSION PROFILE:")
    print("   1. Forward  - 2 seconds")
    print("   2. Right    - 2 seconds")
    print("   3. Backward - 2 seconds")
    print("   4. Left     - 2 seconds")
    print("   5. Forward  - 2 seconds")
    print("=" * 70)
    
    proceed = input("\nReady to proceed? (y/n) [y]: ").strip().lower() or 'y'
    if proceed != 'y':
        print("\n Setup cancelled")
        sys.exit(0)
    
    # Get target height
    while True:
        try:
            height_input = input("\nTakeoff height (0.5-10.0) [1.2]: ").strip()
            target_height = 1.2 if height_input == "" else float(height_input)
            if 0.5 <= target_height <= 10.0:
                print(f" Takeoff height: {target_height}m")
                break
            print("⚠ Must be 0.5-10.0m")
        except ValueError:
            print(" Invalid number")
    
    # Get connection
    connection_string = get_connection_string()
    
    print("\n" + "=" * 70)
    print("STARTING MISSION CONTROL")
    print("=" * 70)
    
    try:
        asyncio.run(mission_loop(connection_string, target_height))
    except KeyboardInterrupt:
        print("\n\n Interrupted by user")
        sys.exit(0)
    except Exception as e:
        print(f"\n ERROR: {e}")
        import traceback
        traceback.print_exc()

        sys.exit(1)

