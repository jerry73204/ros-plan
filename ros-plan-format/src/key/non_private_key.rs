use super::{Key, KeyOwned};
use crate::error::KeyCreationError;
use serde::{Deserialize, Serialize};
use std::{borrow::Borrow, ops::Deref};

#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
#[repr(transparent)]
#[serde(try_from = "KeyOwned", into = "KeyOwned")]
pub struct NonPrivateKeyOwned(pub(crate) KeyOwned);

impl TryFrom<KeyOwned> for NonPrivateKeyOwned {
    type Error = KeyCreationError;

    fn try_from(key: KeyOwned) -> Result<Self, Self::Error> {
        if key.is_private() {
            return Err(KeyCreationError::InitializationError {
                key: key.into(),
                reason: "the key cannot be private".to_string(),
            });
        }

        Ok(Self(key))
    }
}

impl From<NonPrivateKeyOwned> for KeyOwned {
    fn from(key: NonPrivateKeyOwned) -> Self {
        key.0
    }
}

impl Deref for NonPrivateKeyOwned {
    type Target = Key;

    fn deref(&self) -> &Self::Target {
        self.0.deref()
    }
}

impl AsRef<Key> for NonPrivateKeyOwned {
    fn as_ref(&self) -> &Key {
        self.0.as_ref()
    }
}

impl Borrow<Key> for NonPrivateKeyOwned {
    fn borrow(&self) -> &Key {
        self.0.borrow()
    }
}
