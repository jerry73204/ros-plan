use super::{Key, RelativeKeyOwned};
use crate::{
    error::KeyCreationError,
    ident::{Ident, IdentOwned},
};
use serde::{Deserialize, Serialize};
use std::{
    borrow::Borrow,
    fmt::{self, Debug, Display},
    ops::{Deref, Div},
    str::FromStr,
};

#[derive(Clone, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
#[repr(transparent)]
#[serde(try_from = "String", into = "String")]
pub struct KeyOwned(pub(crate) String);

impl KeyOwned {
    pub fn as_key(&self) -> &Key {
        self.as_ref()
    }

    pub fn new_root() -> Self {
        Self("/".to_string())
    }

    pub fn push_ident(&mut self, ident: &Ident) {
        if !self.is_root() {
            self.0.push('/');
        }
        self.0.push_str(ident.as_str());
    }

    pub fn try_push(&mut self, key: &Key) -> Result<(), KeyCreationError> {
        if key.is_absolute() {
            return Err(KeyCreationError::ConcatenationError {
                lhs: self.to_owned(),
                rhs: key.to_string(),
            });
        } else {
            if !self.is_root() {
                self.0.push('/');
            }
            self.0.push_str(key.as_str());
        }
        Ok(())
    }
}

impl TryFrom<String> for KeyOwned {
    type Error = KeyCreationError;

    fn try_from(name: String) -> Result<Self, Self::Error> {
        name.parse()
    }
}

impl FromStr for KeyOwned {
    type Err = KeyCreationError;

    fn from_str(name: &str) -> Result<Self, Self::Err> {
        let key: &Key = name.parse()?;
        Ok(key.to_owned())
    }
}

impl From<KeyOwned> for String {
    fn from(key: KeyOwned) -> Self {
        key.0
    }
}

impl From<IdentOwned> for KeyOwned {
    fn from(ident: IdentOwned) -> Self {
        Self(ident.0)
    }
}

impl TryFrom<KeyOwned> for IdentOwned {
    type Error = KeyOwned;

    fn try_from(key: KeyOwned) -> Result<Self, Self::Error> {
        if key.is_ident() {
            Err(key)
        } else {
            Ok(IdentOwned(key.0))
        }
    }
}

impl Debug for KeyOwned {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        Debug::fmt(self.as_ref(), f)
    }
}

impl Display for KeyOwned {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        Display::fmt(self.as_ref(), f)
    }
}

impl Deref for KeyOwned {
    type Target = Key;

    fn deref(&self) -> &Self::Target {
        unsafe { std::mem::transmute(self.0.as_str()) }
    }
}

impl AsRef<Key> for KeyOwned {
    fn as_ref(&self) -> &Key {
        unsafe { std::mem::transmute(self.0.as_str()) }
    }
}

impl Borrow<Key> for KeyOwned {
    fn borrow(&self) -> &Key {
        unsafe { std::mem::transmute(self.0.as_str()) }
    }
}

impl Div<&str> for &KeyOwned {
    type Output = Result<KeyOwned, KeyCreationError>;

    fn div(self, key: &str) -> Self::Output {
        let rhs: KeyOwned = key.parse()?;
        self.as_ref() / rhs.as_ref()
    }
}

impl Div<&Key> for &KeyOwned {
    type Output = Result<KeyOwned, KeyCreationError>;

    fn div(self, rhs: &Key) -> Self::Output {
        self.as_ref() / rhs
    }
}

impl Div<&Ident> for &KeyOwned {
    type Output = Result<KeyOwned, KeyCreationError>;

    fn div(self, rhs: &Ident) -> Self::Output {
        self.as_ref() / rhs.as_key()
    }
}

impl Div<&IdentOwned> for &KeyOwned {
    type Output = KeyOwned;

    fn div(self, ident: &IdentOwned) -> Self::Output {
        self.as_ref() / ident.as_ref()
    }
}

impl Div<&KeyOwned> for &KeyOwned {
    type Output = Result<KeyOwned, KeyCreationError>;

    fn div(self, rhs: &KeyOwned) -> Self::Output {
        self.as_ref() / rhs.as_ref()
    }
}

impl Div<&RelativeKeyOwned> for &KeyOwned {
    type Output = KeyOwned;

    fn div(self, rhs: &RelativeKeyOwned) -> Self::Output {
        (self / rhs.as_key()).unwrap()
    }
}
