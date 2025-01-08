use crate::error::InvalidRosType;
use regex::Regex;
use serde::{Deserialize, Serialize};
use std::{borrow::Borrow, ops::Deref, str::FromStr, sync::LazyLock};

pub static RE_ROS_TYPE: LazyLock<Regex> = LazyLock::new(|| {
    Regex::new("^[a-zA-Z0-9_]+/(msg|srv|action)/[a-zA-Z0-9_]+$").expect("invalid regex for key")
});

#[derive(Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct RosType(str);

impl RosType {
    pub fn package(&self) -> &str {
        let (package, _) = self.0.split_once('/').unwrap();
        package
    }

    pub fn kind(&self) -> RosTypeKind {
        let kind_str = self.0.split('/').skip(1).take(1).next().unwrap();
        match kind_str {
            "msg" => RosTypeKind::Msg,
            "srv" => RosTypeKind::Srv,
            "action" => RosTypeKind::Action,
            _ => unreachable!(),
        }
    }

    pub fn ty(&self) -> &str {
        let (_, ty) = self.0.rsplit_once('/').unwrap();
        ty
    }

    pub fn as_str(&self) -> &str {
        &self.0
    }
}

impl FromStr for &RosType {
    type Err = InvalidRosType;

    fn from_str(text: &str) -> Result<Self, Self::Err> {
        if !RE_ROS_TYPE.is_match(text) {
            return Err(InvalidRosType {
                bad_type: text.to_string(),
                reason: "not a valid ROS type".to_string(),
            });
        }
        let ty: &RosType = unsafe { std::mem::transmute(text) };
        Ok(ty)
    }
}

#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(transparent)]
pub struct RosTypeOwned(String);

impl RosTypeOwned {
    pub fn as_str(&self) -> &str {
        &self.0
    }
}

impl AsRef<RosType> for RosTypeOwned {
    fn as_ref(&self) -> &RosType {
        unsafe { std::mem::transmute(self.as_str()) }
    }
}

impl Borrow<RosType> for RosTypeOwned {
    fn borrow(&self) -> &RosType {
        self.as_ref()
    }
}

impl Deref for RosTypeOwned {
    type Target = RosType;

    fn deref(&self) -> &Self::Target {
        self.as_ref()
    }
}

impl FromStr for RosTypeOwned {
    type Err = InvalidRosType;

    fn from_str(text: &str) -> Result<Self, Self::Err> {
        if !RE_ROS_TYPE.is_match(text) {
            return Err(InvalidRosType {
                bad_type: text.to_string(),
                reason: "not a valid ROS type".to_string(),
            });
        }
        Ok(Self(text.to_string()))
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum RosTypeKind {
    Msg,
    Srv,
    Action,
}
