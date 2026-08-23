# Health/status snapshot for a single loaded plugin.

uint8 UNKNOWN=0
uint8 UNCONFIGURED=1
uint8 INACTIVE=2
uint8 ACTIVE=3
uint8 DEGRADED=4
uint8 FAULT=5

string name        # instance name given at load time
string type        # pluginlib lookup name / class
string category    # "sensor" | "actuator" | "brain"
uint8 state        # one of the constants above
string message     # human-readable detail
