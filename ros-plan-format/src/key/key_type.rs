use crate::{
    error::KeyCreationError,
    ident::{Ident, IdentOwned},
};
use std::{
    fmt::{self, Debug, Display},
    ops::Div,
    str::FromStr,
};

use super::{KeyOwned, RE_KEY};

#[derive(PartialEq, Eq, PartialOrd, Ord, Hash)]
#[repr(transparent)]
pub struct Key(pub(crate) str);

impl Key {
    pub const fn root() -> &'static Self {
        unsafe { std::mem::transmute("/") }
    }

    pub const fn private_root() -> &'static Self {
        unsafe { std::mem::transmute("~/") }
    }

    pub fn is_absolute(&self) -> bool {
        self.0.starts_with('/')
    }

    pub fn is_private(&self) -> bool {
        self.0.starts_with("~/")
    }

    pub fn is_relative(&self) -> bool {
        !self.is_absolute() && !self.is_private()
    }

    pub fn is_root(&self) -> bool {
        &self.0 == "/"
    }

    pub fn is_private_root(&self) -> bool {
        &self.0 == "~/"
    }

    pub fn is_ident(&self) -> bool {
        !self.0.contains('/')
    }

    pub fn as_str(&self) -> &str {
        &self.0
    }

    pub fn to_kind(&self) -> KeyKind<'_> {
        match self.strip_prefix(Key::root()) {
            StripKeyPrefix::ImproperPrefix => (),
            StripKeyPrefix::EmptySuffix => {
                return KeyKind::Absolute {
                    root: Key::root(),
                    suffix: None,
                }
            }
            StripKeyPrefix::Suffix(suffix) => {
                return KeyKind::Absolute {
                    root: Key::root(),
                    suffix: Some(suffix),
                }
            }
        }

        match self.strip_prefix(Key::private_root()) {
            StripKeyPrefix::ImproperPrefix => (),
            StripKeyPrefix::EmptySuffix => {
                return KeyKind::Private {
                    private_root: Key::private_root(),
                    suffix: None,
                }
            }
            StripKeyPrefix::Suffix(suffix) => {
                return KeyKind::Private {
                    private_root: Key::private_root(),
                    suffix: Some(suffix),
                }
            }
        }

        KeyKind::Relative { key: self }
    }

    pub fn split_root(&self) -> (Option<&Key>, Option<&Key>) {
        let self_str = self.as_str();

        if let Some(suffix) = self_str.strip_prefix("/") {
            let prefix = Some(Key::root());
            let suffix = (!suffix.is_empty()).then(|| suffix.parse().unwrap());
            (prefix, suffix)
        } else if let Some(suffix) = self_str.strip_prefix("~/") {
            let prefix = Some(Key::root());
            let suffix = (!suffix.is_empty()).then(|| suffix.parse().unwrap());
            (prefix, suffix)
        } else {
            (None, Some(self))
        }
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
                let name: &Ident = self
                    .try_into()
                    .expect("the key itself is not a valid identifier");
                (None, Some(name))
            }
        }
    }

    pub fn starts_with(&self, prefix: &Key) -> bool {
        let Some(suffix) = self.as_str().strip_prefix(prefix.as_str()) else {
            return false;
        };

        suffix.is_empty() || suffix.starts_with('/')
    }

    pub fn strip_prefix(&self, prefix: &Key) -> StripKeyPrefix<'_> {
        let suffix_with_slash = self.as_str().strip_prefix(prefix.as_str());
        let Some(suffix_with_slash) = suffix_with_slash else {
            return StripKeyPrefix::ImproperPrefix;
        };
        if suffix_with_slash.is_empty() {
            StripKeyPrefix::EmptySuffix
        } else if let Some(suffix) = suffix_with_slash.strip_prefix('/') {
            let suffix: &Key = suffix.parse().unwrap();
            StripKeyPrefix::Suffix(suffix)
        } else {
            StripKeyPrefix::ImproperPrefix
        }
    }

    pub fn components(&self) -> impl Iterator<Item = &Ident> + Debug + Clone + '_ {
        let suffix = match self.to_kind() {
            KeyKind::Relative { key } => Some(key),
            KeyKind::Absolute { suffix, .. } => suffix,
            KeyKind::Private { suffix, .. } => suffix,
        };
        suffix
            .into_iter()
            .flat_map(|suffix| suffix.as_str().split('/'))
            .map(|token| {
                let token: &Ident = token.parse().unwrap();
                token
            })
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

pub enum StripKeyPrefix<'a> {
    ImproperPrefix,
    EmptySuffix,
    Suffix(&'a Key),
}

pub enum KeyKind<'a> {
    Relative {
        key: &'a Key,
    },
    Absolute {
        root: &'a Key,
        suffix: Option<&'a Key>,
    },
    Private {
        private_root: &'a Key,
        suffix: Option<&'a Key>,
    },
}
