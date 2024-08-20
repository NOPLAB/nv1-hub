use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, Default)]
pub struct JetsonMsgPackCmdVel {
    pub linear_x: f32,
    pub linear_y: f32,
    pub angular_z: f32,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, Default)]
pub struct JetsonMsgPack {
    pub cmd_vel: JetsonMsgPackCmdVel,
}
