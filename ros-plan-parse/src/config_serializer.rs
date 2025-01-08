use crate::{
    context::{GlobalContextV2, LinkContext, NodeTopicUri, PubSubLinkContext, ServiceLinkContext},
    utils::ArcRwLock,
};
use by_address::ByAddress;
use indexmap::IndexMap;
use ros_plan_format::{
    ident::IdentOwned,
    key::{Key, KeyOwned, NonEmptyRelativeKeyOwned},
    link::{Link, LinkIdent, LinkTable, PubSubLink, ServiceLink, TopicUri},
    node::{Node, NodeIdent, NodeTable},
    socket::SocketTable,
    subplan::{HerePlan, Subplan, SubplanTable},
    Plan,
};
use std::{
    collections::{HashMap, VecDeque},
    marker::PhantomData,
};

pub struct ConfigSerializer {
    _private: PhantomData<()>,
}

impl ConfigSerializer {
    pub fn new() -> Self {
        Self {
            _private: PhantomData,
        }
    }

    pub fn convert(&self, context: &GlobalContextV2) -> Plan {
        let trie = DataTrie::default();
        let rev_node_map: HashMap<_, _> = context
            .node_map
            .iter()
            .map(|(key, node_weak)| {
                let node_arc = node_weak.upgrade().unwrap();
                (ByAddress(node_arc.into_arc()), key)
            })
            .collect();

        for (key, node_weak) in &context.node_map {
            let (Some(parent), Some(name)) = key.split_parent() else {
                unreachable!()
            };

            let node_cfg = {
                let node_arc = node_weak.upgrade().unwrap();
                let guard = node_arc.read();
                guard.config.clone()
            };

            let data_node = trie.get_or_insert_key(parent);
            let mut guard = data_node.write();
            guard.node_map.insert(name.to_owned(), node_cfg);
        }

        let resolve_uri = |uri: &NodeTopicUri| -> TopicUri {
            let NodeTopicUri { node, topic } = uri;
            let node_arc = node.upgrade().unwrap();
            let node_arc = ByAddress(node_arc.into_arc());
            let key = rev_node_map[&node_arc];
            let key: KeyOwned = key.to_owned();
            TopicUri {
                node: key.try_into().unwrap(),
                topic: topic.clone(),
            }
        };

        for (key, link_weak) in &context.link_map {
            let (Some(parent), Some(name)) = key.split_parent() else {
                unreachable!()
            };

            let link_cfg = {
                let link_arc = link_weak.upgrade().unwrap();
                let link_ctx = link_arc.read();

                let link: Link = match &*link_ctx {
                    LinkContext::PubSub(pubsub_ctx) => {
                        let PubSubLinkContext { ty, qos, src, dst } = pubsub_ctx;

                        let src: Vec<_> = src.iter().map(|uri| resolve_uri(uri)).collect();
                        let dst: Vec<_> = dst.iter().map(|uri| resolve_uri(uri)).collect();

                        PubSubLink {
                            ty: ty.clone(),
                            qos: qos.clone(),
                            src,
                            dst,
                        }
                        .into()
                    }
                    LinkContext::Service(service_ctx) => {
                        let ServiceLinkContext {
                            ty,
                            listen,
                            connect,
                        } = service_ctx;

                        let listen = resolve_uri(listen);
                        let connect: Vec<_> = connect.iter().map(|uri| resolve_uri(uri)).collect();

                        ServiceLink {
                            ty: ty.clone(),
                            listen,
                            connect,
                        }
                        .into()
                    }
                };

                link
            };

            let data_node = trie.get_or_insert_key(parent);
            let mut guard = data_node.write();
            guard.link_map.insert(name.to_owned(), link_cfg);
        }

        {
            struct Job {
                plan_parent: DataNodeRef,
                current: DataNodeRef,
                prefix: KeyOwned,
                suffix: KeyOwned,
            }

            let mut queue = VecDeque::new();

            {
                let guard = trie.root.read();
                let jobs = guard.children.iter().map(|(ident, child)| Job {
                    plan_parent: trie.root.clone(),
                    current: child.clone(),
                    prefix: KeyOwned::new_root(),
                    suffix: ident.to_key(),
                });
                queue.extend(jobs);
            }

            while let Some(job) = queue.pop_front() {
                let Job {
                    plan_parent,
                    current,
                    prefix,
                    suffix,
                } = job;

                let has_node_or_link = {
                    let guard = current.read();
                    guard.has_node_or_link()
                };

                if has_node_or_link {
                    let mut guard = plan_parent.write();
                    guard.subplan_keys.push((suffix.clone(), current.clone()));
                }

                let (next_parent, next_prefix, next_suffix) = if has_node_or_link {
                    let new_parent = current.clone();
                    let Ok(new_prefix) = &prefix / &suffix else {
                        unreachable!()
                    };
                    let new_suffix = KeyOwned::new_empty();
                    (new_parent, new_prefix, new_suffix)
                } else {
                    (plan_parent, prefix, suffix)
                };

                {
                    let guard = current.read();
                    let jobs = guard.children.iter().map(|(ident, child)| Job {
                        plan_parent: next_parent.clone(),
                        current: child.clone(),
                        prefix: next_prefix.clone(),
                        suffix: &next_suffix / ident,
                    });
                    queue.extend(jobs);
                }
            }
        }

        let root_hereplan = {
            fn visit(current: DataNodeRef) -> HerePlan {
                let mut guard = current.write();

                let subplan: IndexMap<NonEmptyRelativeKeyOwned, Subplan> = guard
                    .subplan_keys
                    .drain(..)
                    .map(|(key, child)| {
                        let hereplan = visit(child);
                        (key.try_into().unwrap(), hereplan.into())
                    })
                    .collect();
                HerePlan {
                    subplan: SubplanTable(subplan),
                    node: NodeTable(std::mem::take(&mut guard.node_map)),
                    link: LinkTable(std::mem::take(&mut guard.link_map)),
                    when: None,
                }
            }

            visit(trie.root)
        };

        let HerePlan {
            subplan,
            node,
            link,
            when: _,
        } = root_hereplan;
        let root_plan = Plan {
            subplan,
            socket: SocketTable::default(),
            arg: IndexMap::new(),
            var: IndexMap::new(),
            node,
            link,
        };

        root_plan
    }
}

type DataNodeRef = ArcRwLock<DataNode>;

#[derive(Debug)]
struct DataTrie {
    root: DataNodeRef,
}

impl Default for DataTrie {
    fn default() -> Self {
        Self {
            root: DataNode::default().into(),
        }
    }
}

impl DataTrie {
    pub fn get_or_insert_key(&self, key: &Key) -> DataNodeRef {
        assert!(key.is_absolute());

        let mut curr = self.root.clone();
        for comp in key.components() {
            let child: DataNodeRef = {
                let mut guard = curr.write();
                guard
                    .children
                    .entry(comp.to_owned())
                    .or_insert_with(|| DataNode::default().into())
                    .clone()
            };
            curr = child;
        }

        curr
    }
}

#[derive(Debug)]
struct DataNode {
    subplan_keys: Vec<(KeyOwned, DataNodeRef)>,
    children: HashMap<IdentOwned, DataNodeRef>,
    node_map: IndexMap<NodeIdent, Node>,
    link_map: IndexMap<LinkIdent, Link>,
}

impl DataNode {
    pub fn has_node_or_link(&self) -> bool {
        !self.node_map.is_empty() || !self.link_map.is_empty()
    }
}

impl Default for DataNode {
    fn default() -> Self {
        Self {
            children: HashMap::new(),
            node_map: IndexMap::new(),
            link_map: IndexMap::new(),
            subplan_keys: vec![],
        }
    }
}
