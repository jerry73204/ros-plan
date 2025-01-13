use crate::{
    error::{InvalidKeyPrefixError, KeyCreationError},
    ident::{Ident, IdentOwned},
};
use regex::Regex;
use serde::{Deserialize, Serialize};
use std::{
    borrow::Borrow,
    fmt::{self, Debug, Display},
    ops::{Deref, Div},
    str::FromStr,
    sync::LazyLock,
};

pub static RE_KEY: LazyLock<Regex> = LazyLock::new(|| {
    Regex::new("^/?([[:word:]]+(/[[:word:]]+)*)?$").expect("invalid regex for key")
});

#[derive(PartialEq, Eq, PartialOrd, Ord, Hash)]
#[repr(transparent)]
pub struct Key(pub(crate) str);

impl Key {
    pub const fn root() -> &'static Self {
        unsafe { std::mem::transmute("/") }
    }

    pub const fn empty() -> &'static Self {
        unsafe { std::mem::transmute("") }
    }

    pub fn is_absolute(&self) -> bool {
        self.0.starts_with('/')
    }

    pub fn is_root(&self) -> bool {
        &self.0 == "/"
    }

    pub fn is_empty(&self) -> bool {
        self.0.is_empty()
    }

    pub fn is_ident(&self) -> bool {
        !self.0.contains('/')
    }

    pub fn as_str(&self) -> &str {
        &self.0
    }

    pub fn split_parent(&self) -> (Option<&Key>, Option<&Ident>) {
        match self.0.rsplit_once('/') {
            Some((parent, name)) => {
                let parent = if parent.is_empty() {
                    Key::root()
                } else {
                    parent
                        .parse()
                        .expect("the parent of key is not a valid key")
                };
                let name = name
                    .parse()
                    .expect("the last component of the key is not a valid identifier");
                (Some(parent), Some(name))
            }
            None => {
                let name = if self.is_empty() {
                    None
                } else {
                    Some(
                        self.try_into()
                            .expect("the key itself is not a valid identifier"),
                    )
                };
                (None, name)
            }
        }
    }

    pub fn starts_with(&self, prefix: &Key) -> Result<bool, InvalidKeyPrefixError> {
        if self.is_absolute() != prefix.is_absolute() {
            return Err(InvalidKeyPrefixError {
                checked: self.to_owned(),
                prefix: prefix.to_owned(),
            });
        }

        let me = self.as_str();
        let prefix = prefix.as_str();

        let Some(suffix) = me.strip_prefix(prefix) else {
            return Ok(false);
        };

        Ok(suffix.is_empty() || suffix.starts_with('/'))
    }

    pub fn strip_prefix(&self, prefix: &Key) -> Result<Option<&Key>, InvalidKeyPrefixError> {
        if self.is_absolute() != prefix.is_absolute() {
            return Err(InvalidKeyPrefixError {
                checked: self.to_owned(),
                prefix: prefix.to_owned(),
            });
        }

        let Some(suffix) = self.as_str().strip_prefix(prefix.as_str()) else {
            return Ok(None);
        };
        let suffix = if suffix.is_empty() {
            suffix
        } else if let Some(suffix) = suffix.strip_prefix('/') {
            suffix
        } else {
            return Ok(None);
        };

        Ok(Some(suffix.parse().unwrap()))
    }

    pub fn components(&self) -> Box<dyn Iterator<Item = &Ident> + '_> {
        let suffix = match self.0.strip_prefix("/") {
            Some(suffix) => suffix,
            None => &self.0,
        };

        if suffix.is_empty() {
            Box::new(std::iter::empty())
        } else {
            let iter = suffix.split('/').map(|comp: &str| {
                let ident: &Ident = comp.parse().unwrap();
                ident
            });
            Box::new(iter)
        }
    }
}

impl ToOwned for Key {
    type Owned = KeyOwned;

    fn to_owned(&self) -> Self::Owned {
        KeyOwned(self.0.to_string())
    }
}

impl Debug for Key {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        Debug::fmt(&self.0, f)
    }
}

impl Display for Key {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        Display::fmt(&self.0, f)
    }
}

impl Div<&str> for &Key {
    type Output = Result<KeyOwned, KeyCreationError>;

    fn div(self, key: &str) -> Self::Output {
        let rhs: KeyOwned = key.parse()?;
        self / rhs.as_ref()
    }
}

impl Div<&Key> for &Key {
    type Output = Result<KeyOwned, KeyCreationError>;

    fn div(self, rhs: &Key) -> Self::Output {
        let mut lhs = self.to_owned();
        lhs.try_push(rhs)?;
        Ok(lhs)
    }
}

impl Div<&Ident> for &Key {
    type Output = KeyOwned;

    fn div(self, ident: &Ident) -> Self::Output {
        let mut lhs = self.to_owned();
        lhs.push_ident(ident);
        lhs
    }
}

impl Div<&IdentOwned> for &Key {
    type Output = KeyOwned;

    fn div(self, ident: &IdentOwned) -> Self::Output {
        self / ident.as_ref()
    }
}

impl Div<&KeyOwned> for &Key {
    type Output = Result<KeyOwned, KeyCreationError>;

    fn div(self, rhs: &KeyOwned) -> Self::Output {
        let mut lhs = self.to_owned();
        lhs.try_push(rhs.as_ref())?;
        Ok(lhs)
    }
}

impl From<&Ident> for &Key {
    fn from(ident: &Ident) -> Self {
        unsafe { std::mem::transmute(ident) }
    }
}

impl FromStr for &Key {
    type Err = KeyCreationError;

    fn from_str(text: &str) -> Result<Self, Self::Err> {
        if !RE_KEY.is_match(text) {
            return Err(KeyCreationError::InitializationError {
                key: text.to_string(),
                reason: "the key must be non-empty words separated by \
                         '/' consisting of English letters and digits"
                    .to_string(),
            });
        }

        let key: &Key = unsafe { std::mem::transmute(text) };
        Ok(key)
    }
}

#[derive(Clone, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
#[repr(transparent)]
#[serde(try_from = "String", into = "String")]
pub struct KeyOwned(pub(crate) String);

impl KeyOwned {
    pub fn new_root() -> Self {
        Self("/".to_string())
    }

    pub fn new_empty() -> Self {
        Self("".to_string())
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
        } else if !key.is_empty() {
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
        let name = name.to_string();

        if RE_KEY.is_match(&name) {
            Ok(Self(name))
        } else {
            Err(KeyCreationError::InitializationError {
                key: name,
                reason: "the key must be non-empty words separated by \
                         '/' consisting of English letters and digits"
                    .to_string(),
            })
        }
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

#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
#[repr(transparent)]
#[serde(try_from = "KeyOwned", into = "KeyOwned")]
pub struct NonEmptyRelativeKeyOwned(pub(crate) KeyOwned);

impl TryFrom<KeyOwned> for NonEmptyRelativeKeyOwned {
    type Error = KeyCreationError;

    fn try_from(key: KeyOwned) -> Result<Self, Self::Error> {
        if key.is_absolute() {
            return Err(KeyCreationError::InitializationError {
                key: key.into(),
                reason: "the key cannot be absolute".to_string(),
            });
        } else if key.is_empty() {
            return Err(KeyCreationError::InitializationError {
                key: key.into(),
                reason: "the key cannot be empty".to_string(),
            });
        }

        Ok(Self(key))
    }
}

impl From<NonEmptyRelativeKeyOwned> for KeyOwned {
    fn from(key: NonEmptyRelativeKeyOwned) -> Self {
        key.0
    }
}

impl Deref for NonEmptyRelativeKeyOwned {
    type Target = Key;

    fn deref(&self) -> &Self::Target {
        self.0.deref()
    }
}

impl AsRef<Key> for NonEmptyRelativeKeyOwned {
    fn as_ref(&self) -> &Key {
        self.0.as_ref()
    }
}

impl Borrow<Key> for NonEmptyRelativeKeyOwned {
    fn borrow(&self) -> &Key {
        self.0.borrow()
    }
}

#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
#[repr(transparent)]
#[serde(try_from = "KeyOwned", into = "KeyOwned")]
pub struct NonEmptyKeyOwned(pub(crate) KeyOwned);

impl TryFrom<KeyOwned> for NonEmptyKeyOwned {
    type Error = KeyCreationError;

    fn try_from(key: KeyOwned) -> Result<Self, Self::Error> {
        if key.is_empty() || key.is_root() {
            return Err(KeyCreationError::InitializationError {
                key: key.into(),
                reason: "the key cannot be empty".to_string(),
            });
        }

        Ok(Self(key))
    }
}

impl From<NonEmptyKeyOwned> for KeyOwned {
    fn from(key: NonEmptyKeyOwned) -> Self {
        key.0
    }
}

impl Deref for NonEmptyKeyOwned {
    type Target = Key;

    fn deref(&self) -> &Self::Target {
        self.0.deref()
    }
}

impl AsRef<Key> for NonEmptyKeyOwned {
    fn as_ref(&self) -> &Key {
        self.0.as_ref()
    }
}

impl Borrow<Key> for NonEmptyKeyOwned {
    fn borrow(&self) -> &Key {
        self.0.borrow()
    }
}

#[cfg(test)]
mod tests {
    use super::KeyOwned;
    use std::str::FromStr;

    #[test]
    fn suceed_on_valid_keys() {
        let valid_keys = [
            "",
            "/",
            "/x",
            "/xy",
            "/x/y",
            "/xx/y/zzz",
            "xyz",
            "xy",
            "x/y/z",
        ];

        for path in valid_keys {
            let _: KeyOwned = path.parse().unwrap();
        }
    }

    #[test]
    fn fail_on_invalid_keys() {
        let invalid_keys = ["/x/", "//", "/x//y", "xx//y", "xx//", "yy/"];

        for path in invalid_keys {
            let Err(_) = KeyOwned::from_str(path) else {
                panic!("the key '{path}' is invalid but the Key is successfully constructed")
            };
        }
    }
}
