//! [`Prefix`], the entity-path prefix every recording of this crate is written under.

use rerun::EntityPath;

/// What every entity path of one recorder is put under: with the prefix `L`, `joints/q`
/// becomes `L/joints/q`. Two robots can then write into one Rerun recording without their
/// series landing on each other; the empty prefix ([`Prefix::none`], the default) changes
/// nothing, which is what a single-robot replay uses.
///
/// The name is an entity path part and is taken as given -- the node passes the arm's name,
/// which its configuration has already checked to be `[A-Za-z0-9_-]+`.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct Prefix {
    /// Empty, or the name with the separator: `"L/"`.
    under: String,
    name: String,
}

impl Prefix {
    /// The prefix `name`; an empty `name` is [`Prefix::none`].
    pub fn new(name: impl Into<String>) -> Prefix {
        let name = name.into();
        let under = if name.is_empty() {
            String::new()
        } else {
            format!("{name}/")
        };
        Prefix { under, name }
    }

    /// No prefix: paths are unchanged.
    pub fn none() -> Prefix {
        Prefix::default()
    }

    /// The name, empty when there is none.
    pub fn name(&self) -> &str {
        &self.name
    }

    pub fn is_empty(&self) -> bool {
        self.name.is_empty()
    }

    /// `<prefix>/<tail>`, or `tail` alone without a prefix.
    pub fn path(&self, tail: &str) -> String {
        format!("{}{tail}", self.under)
    }

    /// [`Prefix::path`] as an entity path, for a caller that builds one once and logs to it
    /// many times.
    pub fn entity(&self, tail: &str) -> EntityPath {
        EntityPath::from(self.path(tail))
    }

    /// [`Prefix::path`] rooted, which is how a blueprint names an entity it does not own:
    /// `/L/ee/position/x`.
    pub fn rooted(&self, tail: &str) -> String {
        format!("/{}{tail}", self.under)
    }

    /// `what` with the name in front when there is one, for a view's title: `"L arm"`.
    pub fn label(&self, what: &str) -> String {
        if self.is_empty() {
            what.to_string()
        } else {
            format!("{} {what}", self.name)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn no_prefix_leaves_a_path_alone() {
        let none = Prefix::none();
        assert!(none.is_empty());
        assert_eq!(none.name(), "");
        assert_eq!(none.path("joints/q"), "joints/q");
        assert_eq!(none.rooted("joints/q"), "/joints/q");
        assert_eq!(none.label("arm"), "arm");
        assert_eq!(Prefix::new(""), none);
        assert_eq!(Prefix::default(), none);
    }

    #[test]
    fn a_prefix_is_one_path_part_in_front() {
        let left = Prefix::new("L");
        assert!(!left.is_empty());
        assert_eq!(left.name(), "L");
        assert_eq!(left.path("joints/q"), "L/joints/q");
        assert_eq!(left.path("events"), "L/events");
        assert_eq!(left.rooted("ee/position/x"), "/L/ee/position/x");
        assert_eq!(left.label("arm"), "L arm");
        assert_eq!(left.entity("world/arm"), EntityPath::from("L/world/arm"));
    }
}
