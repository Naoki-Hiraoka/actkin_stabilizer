int64 KEEP_PREV=0
int64 NOT_CARED=1
int64 REL_HRPSYS_ODOM=2
int64 REL_COG=3
int64 TO_CONTACT=4
int64 TO_AIR=5
int64 CONTACT=6

int64 EE_MIDIUM=0
int64 EE_LOW=1
int64 EE_HIGH=2

string name
actkin_stabilizer/EEPoint[] goal
int64 mode
uint32 mode_id
bool[6] is_free
int64 priority
bool[6] is_passive
geometry_msgs/Pose act_pose
