#!/usr/bin/env python3
"""
PX4 Pre-Planned Mission Control
Executes a 5-pointed star pattern mission at 50Hz
- Forward 2s → Rotate 144° → Forward 2s → Rotate 144° → ... (5 times)
- Then asks to repeat or land
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
        self.control_rate = 15  # Hz
        self.time_step = 1.0 / self.control_rate  # 0.02 seconds
        
        # Movement speeds
        self.forward_speed = 1.0   # m/s
        self.yaw_speed = 22.0      # deg/s for rotation (144° in 2 seconds)
        self.thrust = 0.0         # Maintain altitude
        
    def generate_mission(self):
        """
        Generate star pattern mission: 5 edges with 144° turns
        Each edge: Forward 2s → Rotate 144° right in 2s
        Returns list of control values at 50Hz
        """
        mission = []
        
        # Each segment is 2 seconds = 30 control points at 50Hz
        segment_points = int(2.0 * self.control_rate)
        
        # 5 edges of the star
        for edge in range(5):
            edge_num = edge + 1
            
            # Forward segment (2 seconds)
            print(f"Planning: Edge {edge_num} - Forward 2 seconds...")
            for i in range(segment_points):
                mission.append({
                    'pitch': self.forward_speed,   # Forward
                    'roll': 0.0,
                    'thrust': self.thrust,
                    'yaw': 0.0,                    # No rotation while moving
                    'time': len(mission) * self.time_step,
                    'phase': f'EDGE_{edge_num}_FWD'
                })
            
            # Rotation segment (2 seconds) - 144° right
            print(f"Planning: Edge {edge_num} - Rotate 144° right...")
            for i in range(segment_points):
                mission.append({
                    'pitch': 0.0,                  # No forward movement
                    'roll': 0.0,
                    'thrust': self.thrust,
                    'yaw': self.yaw_speed,         # Rotate right at 72°/s
                    'time': len(mission) * self.time_step,
                    'phase': f'EDGE_{edge_num}_ROT'
                })
        
        # Final hold (2 seconds)
        print("Planning: Final hold 2 seconds...")
        for i in range(segment_points):
            mission.append({
                'pitch': 0.0,
                'roll': 0.0,
                'thrust': self.thrust,
                'yaw': 0.0,
                'time': len(mission) * self.time_step,
                'phase': 'HOLD'
            })
        
        print(f"\n Mission planned: {len(mission)} control points ({len(mission) * self.time_step:.1f} seconds at 50Hz)")
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
    print("PX4 PRE-PLANNED MISSION CONTROL - 5-POINTED STAR")
    print("=" * 70)
    print(f"Target Takeoff Height: {target_height}m")
    print("Mission: 5-Edge Star Pattern (Forward→Rotate 144°) × 5")
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
    #async for health in drone.telemetry.health():
        #if health.is_global_position_ok and health.is_home_position_ok:
            #print(" Position estimation ready")
            #break
    
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
    print("\nStabilizing for 2 seconds...")
    await asyncio.sleep(2.0)
    
    # GENERATE MISSION PLAN
    print("\n[7/7] Generating mission plan...")
    
    planner = MissionPlanner()
    mission_plan = planner.generate_mission()
    
    # START OFFBOARD MODE
    print("\nStarting OFFBOARD mode...")
    
    # Set initial setpoint
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
        await drone.action.return_to_launch()  # land()
        return False
    
    print("\n" + "=" * 70)
    print("EXECUTING STAR PATTERN MISSION")
    print("=" * 70)
    
    # EXECUTE MISSION AT 15Hz
    
    mission_start_time = time.time()
    
    try:
        for idx, control_point in enumerate(mission_plan):
            # Send control command
            await drone.offboard.set_velocity_body(
                VelocityBodyYawspeed(
                    forward_m_s=control_point['pitch'],
                    right_m_s=control_point['roll'],
                    down_m_s=-control_point['thrust'],  # Negative for NED frame
                    yawspeed_deg_s=control_point['yaw']
                )
            )
            
            # Display status every 25 points (0.5s intervals)
            if idx % 25 == 0:
                # Get current altitude
                current_altitude = target_height
                async for position in drone.telemetry.position():
                    current_altitude = position.relative_altitude_m
                    break
                
                elapsed = time.time() - mission_start_time
                progress = (idx / len(mission_plan)) * 100
                
                print(f"[{control_point['phase']:12s}] "
                      f"Time: {elapsed:.1f}s | "
                      f"Alt: {current_altitude:.2f}m | "
                      f"Pitch: {control_point['pitch']:+.2f} | "
                      f"Yaw: {control_point['yaw']:+.1f}°/s | "
                      f"Progress: {progress:.0f}%",
                      end='\r')
            
            # Wait for next control point (50Hz = 0.02s)
            await asyncio.sleep(0.02)
    
    except KeyboardInterrupt:
        print("\n\n Mission interrupted by user")
        await drone.action.return_to_launch()
        return False
    except Exception as e:
        print(f"\n\n Error during mission: {e}")
        await drone.action.return_to_launch()
        return False
    
    mission_duration = time.time() - mission_start_time
    print(f"\n\n Mission completed in {mission_duration:.2f} seconds")
    
    # Return to hover
    print("\nReturning to hover position...")
    await drone.offboard.set_velocity_body(
        VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0)
    )
    await asyncio.sleep(1.0)
    
    return True


async def mission_loop(connection_string, target_height):
    """
    Main mission loop - repeat or land
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
        
        # Ask user to repeat or land
        print("\n" + "=" * 70)
        print("MISSION COMPLETE - OPTIONS")
        print("=" * 70)

        repeat = input("\nLand or Return to Launch? (L/R) [R]: ").strip().lower()
        
        if repeat == 'L' or repeat == 'l':
            print("\nUser selected to land")
            await drone.action.land()
            await asyncio.sleep(5.0)
            break
        else:
            print("\nReturn to Launch...")
            await drone.action.return_to_launch()
            await asyncio.sleep(5.0)
            break
    
    # LANDING SEQUENCE    
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
    print("[Landing] Initiating return to launch...")
    await drone.action.return_to_launch()
    
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
    print("PX4 PRE-PLANNED MISSION CONTROL")
    print("5-Pointed Star Pattern Flight - 50Hz Control Rate")
    print("=" * 70)
    
    print("\n DEPENDENCIES:")
    print("   pip install mavsdk")
    print("\n PX4 PARAMETERS:")
    print("   - COM_RCL_EXCEPT = 4")
    print("   - COM_OF_LOSS_T = 1.0")
    print("\n MISSION PROFILE:")
    print("   5-Edge Star Pattern:")
    print("   - Edge 1: Forward 2s → Rotate 144° right (2s)")
    print("   - Edge 2: Forward 2s → Rotate 144° right (2s)")
    print("   - Edge 3: Forward 2s → Rotate 144° right (2s)")
    print("   - Edge 4: Forward 2s → Rotate 144° right (2s)")
    print("   - Edge 5: Forward 2s → Rotate 144° right (2s)")
    print("   - Hold: 2 seconds")
    print("   Total: 22 seconds at 50Hz (1100 control points)")
    print("=" * 70)
    
    proceed = input("\nReady to proceed? (y/n) [y]: ").strip().lower() or 'y'
    if proceed != 'y':
        print("\n Setup cancelled")
        sys.exit(0)
    
    # Get target height
    while True:
        try:
            height_input = input("\nTakeoff height (0.5-10.0) [5]: ").strip()
            target_height = 5.0 if height_input == "" else float(height_input)
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
