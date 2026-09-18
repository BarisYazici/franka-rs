//! The JSON of `franka/<arm>/params/{schema,get,set}` and `franka/<arm>/params/current`: the
//! vocabulary a panel tunes a running arm with.
//!
//! Every bound, default, unit, policy, danger flag and slider range served here is read out of
//! [`LiveTuning::BOUNDS`] at the moment of the reply. There is no table in this file and there
//! must never be one: the schema is a *serialisation of the gate*, so a panel that renders it
//! cannot draw a slider the arm would refuse, and a limit moved in the library moves here with
//! it. The node adds only what the library cannot know -- which arm, which boot, and the
//! read-only `derived` block a client needs to check its own limits against.
//!
//! The node owns the wire and the library owns the domain. Shape errors (`type`, `length`,
//! `unknown_field`, `non_finite`) are decided here, because they are questions about JSON;
//! everything else -- what clamps, what is refused outright -- is
//! [`LiveTuning::apply_update`]'s answer, reported as `invalid`.

use std::ops::Range;

use franka::robot::target_control::{FieldBound, LiveTuning};
use serde::{Deserialize, Serialize};

/// The schema version this module speaks; a consumer refuses anything else.
pub const SCHEMA_VERSION: u32 = 1;
/// Who publishes it. The teleop client publishes the same shape under `"teleop"`.
pub const OWNER: &str = "node";

/// The arm name and boot id of the committed dump (`schema/params-schema.json`), which stands
/// for no particular arm and no particular run: a consumer of the dump substitutes its own, and
/// fixing them here is what makes the dump byte-stable and so checkable.
pub const DUMP_ARM: &str = "L";
/// See [`DUMP_ARM`].
pub const DUMP_BOOT_ID: &str = "dump";

/// One published field: the consecutive [`LiveTuning::BOUNDS`] rows that share a name, which is
/// one scalar or one array on the wire.
pub struct Field {
    /// The field's name, as the schema, `get` and a `set` request all spell it.
    pub name: &'static str,
    /// Its words, in order, as offsets into the table it was read from -- which for
    /// [`fields`] is [`LiveTuning::to_words`] itself.
    pub words: Range<usize>,
    rows: &'static [FieldBound],
}

impl Field {
    fn bounds(&self) -> &'static [FieldBound] {
        self.rows
    }

    /// `f64` for a scalar, `f64[N]` for an array.
    fn kind(&self) -> String {
        match self.rows.len() {
            1 if self.rows[0].index.is_none() => "f64".to_string(),
            n => format!("f64[{n}]"),
        }
    }

    /// `values` of this field's words, as the wire carries them.
    fn value(&self, values: &[f64]) -> ParamValue {
        self.wire(values[self.words.clone()].to_vec())
    }

    /// One number per word, as a scalar where the field is one.
    fn wire(&self, mut of: Vec<f64>) -> ParamValue {
        match self.kind().as_str() {
            "f64" => ParamValue::Scalar(of.remove(0)),
            _ => ParamValue::Array(of),
        }
    }

    /// `of` per word where every word of the field has one, else `None`: a presentation hint
    /// only half an array carries is one no caller can render.
    fn every(&self, of: impl Fn(&FieldBound) -> Option<f64>) -> Option<ParamValue> {
        let values: Option<Vec<f64>> = self.bounds().iter().map(of).collect();
        values.map(|values| self.wire(values))
    }
}

/// The published fields, in the order [`LiveTuning::BOUNDS`] lists their words.
///
/// Walked rather than written out: a field added to the table is published, named and settable
/// without a line changing here, which is the whole reason the table is the only vocabulary.
pub fn fields() -> Vec<Field> {
    fields_of(LiveTuning::BOUNDS)
}

/// The fields of `table`: each run of consecutive rows that share a name, with its word offsets
/// within `table`.
///
/// Taking the table as an argument is what lets a test hand this a different one and watch the
/// schema follow -- the property the whole design rests on. [`FieldBound`] cannot be built
/// outside the library, so the only other tables are slices of the one.
pub fn fields_of(table: &'static [FieldBound]) -> Vec<Field> {
    let mut fields: Vec<Field> = Vec::new();
    for (word, bound) in table.iter().enumerate() {
        match fields.last_mut() {
            Some(field) if field.name == bound.name => {
                field.words.end = word + 1;
                field.rows = &table[field.words.clone()];
            }
            _ => fields.push(Field {
                name: bound.name,
                words: word..word + 1,
                rows: &table[word..word + 1],
            }),
        }
    }
    fields
}

/// A scalar field's value, or an array field's.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(untagged)]
pub enum ParamValue {
    Scalar(f64),
    Array(Vec<f64>),
}

/// `to_json` for everything this module replies with.
pub fn to_json<T: Serialize>(message: &T) -> String {
    serde_json::to_string(message).expect("the params messages hold finite numbers and strings")
}

mod schema;
mod values;

pub use schema::{
    field_schemas, CartesianPreset, Derived, FieldSchema, LeashDerived, Relation, SchemaMsg,
};
pub use values::{
    params_of, update_of, Clamped, Origin, ParamsMsg, ParsedUpdate, Reason, Rejection, SetReply,
    SetRequest,
};

#[cfg(test)]
mod tests;
