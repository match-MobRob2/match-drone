# Navigation Status Message for match_nav

# State constants
uint8 IDLE=0
uint8 NAVIGATING=1
uint8 AVOIDING=2
uint8 REACHED_GOAL=3
uint8 ERROR=4

# Current navigation state
uint8 state

# Distance to final goal (meters)
float32 distance_to_goal

# Current waypoint index in path
uint32 current_waypoint_index

# Total waypoints in current path
uint32 total_waypoints

# Human-readable status message
string status_message
