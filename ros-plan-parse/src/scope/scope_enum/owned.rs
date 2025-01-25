use crate::scope::{GroupScopeOwned, PlanFileScopeOwned};

use super::{ScopeReadGuard, ScopeWriteGuard};

#[derive(Debug)]
pub enum ScopeOwned {
    Group(GroupScopeOwned),
    Include(PlanFileScopeOwned),
}

impl ScopeOwned {
    pub fn read(&self) -> ScopeReadGuard<'_> {
        match self {
            ScopeOwned::Group(owned) => owned.read().into(),
            ScopeOwned::Include(owned) => owned.read().into(),
        }
    }

    pub fn write(&self) -> ScopeWriteGuard<'_> {
        match self {
            ScopeOwned::Group(owned) => owned.write().into(),
            ScopeOwned::Include(owned) => owned.write().into(),
        }
    }

    pub fn as_group(&self) -> Option<&GroupScopeOwned> {
        if let Self::Group(v) = self {
            Some(v)
        } else {
            None
        }
    }

    pub fn as_group_mut(&mut self) -> Option<&mut GroupScopeOwned> {
        if let Self::Group(v) = self {
            Some(v)
        } else {
            None
        }
    }

    pub fn as_include(&self) -> Option<&PlanFileScopeOwned> {
        if let Self::Include(v) = self {
            Some(v)
        } else {
            None
        }
    }

    pub fn as_include_mut(&mut self) -> Option<&mut PlanFileScopeOwned> {
        if let Self::Include(v) = self {
            Some(v)
        } else {
            None
        }
    }
}

impl From<PlanFileScopeOwned> for ScopeOwned {
    fn from(v: PlanFileScopeOwned) -> Self {
        Self::Include(v)
    }
}

impl From<GroupScopeOwned> for ScopeOwned {
    fn from(v: GroupScopeOwned) -> Self {
        Self::Group(v)
    }
}
