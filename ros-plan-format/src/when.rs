use crate::{
    expr::BoolExpr,
    link::{PubSubLinkCfg, ServiceLinkCfg},
    node::NodeCfg,
    subplan::{GroupCfg, IncludeCfg},
};

pub trait WithWhen {
    fn when(&self) -> Option<&BoolExpr>;
}

impl WithWhen for IncludeCfg {
    fn when(&self) -> Option<&BoolExpr> {
        self.when.as_ref()
    }
}

impl WithWhen for GroupCfg {
    fn when(&self) -> Option<&BoolExpr> {
        self.when.as_ref()
    }
}

impl WithWhen for NodeCfg {
    fn when(&self) -> Option<&BoolExpr> {
        self.when.as_ref()
    }
}

impl WithWhen for PubSubLinkCfg {
    fn when(&self) -> Option<&BoolExpr> {
        self.when.as_ref()
    }
}

impl WithWhen for ServiceLinkCfg {
    fn when(&self) -> Option<&BoolExpr> {
        self.when.as_ref()
    }
}
