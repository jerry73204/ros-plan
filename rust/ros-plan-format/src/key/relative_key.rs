use super::{Key, KeyOwned};
use crate::{error::KeyCreationError, ident::Ident};
use serde::{Deserialize, Serialize};
use std::{
    borrow::Borrow,
    ops::{Deref, Div},
};

#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
#[repr(transparent)]
#[serde(try_from = "KeyOwned", into = "KeyOwned")]
pub struct RelativeKeyOwned(pub(crate) KeyOwned);

impl RelativeKeyOwned {
    pub fn as_key(&self) -> &Key {
        self.as_ref()
    }
}

impl TryFrom<KeyOwned> for RelativeKeyOwned {
    type Error = KeyCreationError;

    fn try_from(key: KeyOwned) -> Result<Self, Self::Error> {
        if !key.is_relative() {
            return Err(KeyCreationError::InitializationError {
                key: key.into(),
                reason: "the key cannot be absolute".to_string(),
            });
        }

        Ok(Self(key))
    }
}

impl From<RelativeKeyOwned> for KeyOwned {
    fn from(key: RelativeKeyOwned) -> Self {
        key.0
    }
}

impl Deref for RelativeKeyOwned {
    type Target = Key;

    fn deref(&self) -> &Self::Target {
        self.0.deref()
    }
}

impl AsRef<Key> for RelativeKeyOwned {
    fn as_ref(&self) -> &Key {
        self.0.as_ref()
    }
}

impl Borrow<Key> for RelativeKeyOwned {
    fn borrow(&self) -> &Key {
        self.0.borrow()
    }
}

impl Div<&KeyOwned> for &RelativeKeyOwned {
    type Output = Result<RelativeKeyOwned, KeyCreationError>;

    fn div(self, rhs: &KeyOwned) -> Self::Output {
        (self.as_key() / rhs)?.try_into()
    }
}

impl Div<&Key> for &RelativeKeyOwned {
    type Output = Result<RelativeKeyOwned, KeyCreationError>;

    fn div(self, rhs: &Key) -> Self::Output {
        (self.as_key() / rhs)?.try_into()
    }
}

impl Div<&Ident> for &RelativeKeyOwned {
    type Output = Result<RelativeKeyOwned, KeyCreationError>;

    fn div(self, rhs: &Ident) -> Self::Output {
        (self.as_key() / rhs).try_into()
    }
}

impl Div<&str> for &RelativeKeyOwned {
    type Output = Result<RelativeKeyOwned, KeyCreationError>;

    fn div(self, rhs: &str) -> Self::Output {
        (self.as_key() / rhs)?.try_into()
    }
}

impl Div<&RelativeKeyOwned> for &RelativeKeyOwned {
    type Output = RelativeKeyOwned;

    fn div(self, rhs: &RelativeKeyOwned) -> Self::Output {
        let key = format!("{}/{}", self.as_str(), rhs.as_str());
        let key: KeyOwned = key.parse().unwrap();
        key.try_into().unwrap()
    }
}
