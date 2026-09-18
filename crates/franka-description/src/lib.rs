//! The visual meshes of Franka's arms, built in: `link0 .. link7`, the Franka Hand and its
//! finger for the FR3 and the Franka Emika Robot (FER, Panda), as glTF binaries.
//!
//! They are franka_description's meshes (Apache-2.0, Franka Robotics GmbH; see `NOTICE`),
//! converted from Collada and decimated to about [`TRIANGLE_RATIO`] of their triangles by
//! `tools/franka-meshes` in the franka-rs repository; `meshes/SOURCES.md` has the command and
//! the deviation of every mesh. Each mesh is in its link's frame, every glTF node the identity,
//! colours as base colours, no normals.
//!
//! ```
//! use franka_description::{MeshSet, Robot};
//!
//! let fr3 = MeshSet::for_robot(Robot::Fr3);
//! assert_eq!(&fr3.links[7][..4], b"glTF");
//! ```

#![no_std]

/// The franka_description commit the meshes were converted from.
pub const SOURCE_COMMIT: &str = "7aeeddc";

/// The share of each link's triangles the meshes keep; the hand keeps twice that, and the
/// finger and parts under 200 triangles are whole.
pub const TRIANGLE_RATIO: f64 = 0.15;

/// Which arm.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Robot {
    /// Franka Research 3.
    Fr3,
    /// Franka Emika Robot (Panda).
    Fer,
}

/// One robot's meshes, each a complete `.glb`.
#[derive(Clone, Copy, PartialEq, Eq)]
pub struct MeshSet {
    /// `link0` (the base) .. `link7`, each in its link's frame.
    pub links: [&'static [u8]; 8],
    /// The Franka Hand, in the hand frame (the flange yawed by -45 degrees).
    pub hand: &'static [u8],
    /// One finger, in its finger frame.
    pub finger: &'static [u8],
}

macro_rules! glb {
    ($robot:literal, $name:literal) => {
        include_bytes!(concat!("../meshes/", $robot, "/", $name, ".glb")) as &[u8]
    };
}

macro_rules! mesh_set {
    ($robot:literal) => {
        MeshSet {
            links: [
                glb!($robot, "link0"),
                glb!($robot, "link1"),
                glb!($robot, "link2"),
                glb!($robot, "link3"),
                glb!($robot, "link4"),
                glb!($robot, "link5"),
                glb!($robot, "link6"),
                glb!($robot, "link7"),
            ],
            hand: glb!($robot, "hand"),
            finger: glb!($robot, "finger"),
        }
    };
}

/// The FR3's meshes, with the white Franka Hand.
pub const FR3: MeshSet = mesh_set!("fr3");
/// The FER's meshes.
pub const FER: MeshSet = mesh_set!("fer");

impl MeshSet {
    /// The meshes of `robot`.
    pub fn for_robot(robot: Robot) -> &'static MeshSet {
        match robot {
            Robot::Fr3 => &FR3,
            Robot::Fer => &FER,
        }
    }
}

impl core::fmt::Debug for MeshSet {
    /// The sizes, not the bytes.
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("MeshSet")
            .field("links", &self.links.map(<[u8]>::len))
            .field("hand", &self.hand.len())
            .field("finger", &self.finger.len())
            .finish()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn u32_at(bytes: &[u8], at: usize) -> usize {
        u32::from_le_bytes(bytes[at..at + 4].try_into().unwrap()) as usize
    }

    #[test]
    fn every_mesh_is_a_whole_glb_without_normals() {
        for set in [FR3, FER] {
            for glb in set.links.iter().chain([&set.hand, &set.finger]) {
                assert_eq!(&glb[..4], b"glTF");
                assert_eq!(u32_at(glb, 4), 2, "glTF version");
                assert_eq!(u32_at(glb, 8), glb.len(), "header length");
                assert_eq!(&glb[16..20], b"JSON");
                let json = &glb[20..20 + u32_at(glb, 12)];
                assert!(!json.windows(6).any(|w| w == b"NORMAL"));
            }
        }
        assert_eq!(MeshSet::for_robot(Robot::Fer), &FER);
    }
}
