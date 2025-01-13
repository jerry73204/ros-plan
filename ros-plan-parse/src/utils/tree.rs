use std::collections::BTreeMap;

use crate::{error::Error, utils::arc_rwlock::ArcRwLock};
use ros_plan_format::key::{Key, KeyOwned};
use serde::{Deserialize, Serialize};

pub type TreeRef<T> = ArcRwLock<Tree<T>>;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Tree<T> {
    pub value: T,
    pub children: BTreeMap<KeyOwned, TreeRef<T>>,
}

impl<T> Tree<T> {
    pub fn new(value: T) -> Self {
        Self {
            value,
            children: BTreeMap::new(),
        }
    }
}

impl<T> TreeRef<T> {
    pub fn get_child(&self, key: &Key) -> Option<(Self, Option<KeyOwned>)> {
        if key.is_absolute() {
            return None;
        }
        if key.is_empty() {
            return None;
        }

        let guard = self.read();
        let children = &guard.children;

        if let Some((prev_key, prev_child)) = children.range(..=key.to_owned()).next_back() {
            if let Some(suffix) = key.strip_prefix(prev_key).unwrap() {
                let suffix = (!suffix.is_empty()).then(|| suffix.to_owned());
                return Some((prev_child.clone(), suffix));
            }
        }

        None
    }

    pub fn insert(&self, key: KeyOwned, value: T) -> Result<Self, Error> {
        // Reject absolute keys.
        if key.is_absolute() {
            return Err(Error::InvalidSubplanName {
                key: key.to_owned(),
                reason: "absolute key is not allowed".to_string(),
            });
        }

        // Reject empty keys.
        if key.is_empty() {
            return Err(Error::InvalidSubplanName {
                key: key.to_owned(),
                reason: "empty key not allowed".to_string(),
            });
        }

        // Return error if the key is a prefix or an extension of an
        // existing key.
        let mut guard = self.write();
        let children = &mut guard.children;

        if let Some((prev_key, _)) = children.range(..key.clone()).next_back() {
            if key.starts_with(prev_key).unwrap() {
                return Err(Error::ConflictingKeys {
                    offender: prev_key.to_owned(),
                    inserted: key.to_owned(),
                });
            }
        }

        if let Some((next_key, _)) = children.range(key.clone()..).next() {
            if next_key.starts_with(&key).unwrap() {
                return Err(Error::ConflictingKeys {
                    offender: next_key.to_owned(),
                    inserted: key.to_owned(),
                });
            }
        }

        // Create the child
        let child: TreeRef<T> = Tree {
            children: BTreeMap::new(),
            value,
        }
        .into();
        children.insert(key, child.clone());

        Ok(child)
    }
}
