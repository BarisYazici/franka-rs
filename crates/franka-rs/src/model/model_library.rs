//! Downloading the FCI v5 model library from the robot (`LoadModelLibrary`).
//!
//! Port of `franka::LibraryDownloader` (`src/library_downloader.{h,cpp}`,
//! libfranka 0.9.2). On FCI v5 the robot has no URDF to hand out; instead it
//! serves a *compiled* model for the architecture and operating system the
//! client asks for, and libfranka `dlopen`s it. The download is a single
//! command on the ordinary TCP command socket:
//!
//! * request: `u8 architecture, u8 system` (2 bytes),
//! * response: `u8 status`, followed by the shared object as the message tail
//!   (`header.size - 13` bytes: the 12-byte command header plus the status byte
//!   are subtracted).
//!
//! [`load_from_robot`] performs that exchange and hands the bytes to
//! [`SoModelBackend::from_bytes`]; the offline entry points
//! [`crate::model::Model::from_model_library_bytes`] and
//! [`crate::model::Model::from_model_library_path`] skip the network.

#[cfg(feature = "model-library")]
use std::path::Path;

#[cfg(feature = "model-library")]
use zerocopy::IntoBytes;

use crate::error::{FrankaError, FrankaResult};
#[cfg(feature = "model-library")]
use crate::model::so_backend::SoModelBackend;
use crate::model::Model;
use crate::network::Network;
use crate::wire::robot::codec::{command_id, CommandKind, FciVersion};
#[cfg(feature = "model-library")]
use crate::wire::robot::v5;
#[cfg(feature = "model-library")]
use crate::wire::{message_payload, parse_response, HeaderLayout};

/// The wire command id of `LoadModelLibrary` under `version`.
///
/// # Errors
///
/// [`FrankaError::InvalidOperation`] on a version that has no such command,
/// i.e. FCI v10, whose FR3 serves a URDF through `GetRobotModel` instead.
fn load_model_library_command(version: FciVersion) -> FrankaResult<u32> {
    command_id(version, CommandKind::LoadModelLibrary).ok_or_else(|| {
        FrankaError::InvalidOperation(format!(
            "libfranka: {} is not available on FCI version {}.",
            CommandKind::LoadModelLibrary.name(),
            version.number()
        ))
    })
}

/// The `LoadModelLibrary::Architecture` this build asks the robot for.
///
/// libfranka picks the value from the `LIBFRANKA_X64` / `LIBFRANKA_X86` /
/// `LIBFRANKA_ARM64` / `LIBFRANKA_ARM` macros its CMake sets from the target
/// processor (`src/platform.h`, `library_downloader.cpp`); `cfg!(target_arch)`
/// is the direct equivalent.
///
/// # Errors
///
/// [`FrankaError::Model`] with libfranka's own
/// `"libfranka: Unsupported architecture!"` on any other target.
#[cfg(feature = "model-library")]
fn architecture() -> FrankaResult<v5::LoadModelLibraryArchitecture> {
    if cfg!(target_arch = "x86_64") {
        Ok(v5::LoadModelLibraryArchitecture::X64)
    } else if cfg!(target_arch = "x86") {
        Ok(v5::LoadModelLibraryArchitecture::X86)
    } else if cfg!(target_arch = "aarch64") {
        Ok(v5::LoadModelLibraryArchitecture::ARM64)
    } else if cfg!(target_arch = "arm") {
        Ok(v5::LoadModelLibraryArchitecture::ARM)
    } else {
        Err(FrankaError::Model(
            "libfranka: Unsupported architecture!".to_string(),
        ))
    }
}

/// The `LoadModelLibrary::System` this build asks the robot for.
///
/// # Errors
///
/// [`FrankaError::Model`] with libfranka's own
/// `"libfranka: Unsupported operating system!"` on any other target.
#[cfg(feature = "model-library")]
fn system() -> FrankaResult<v5::LoadModelLibrarySystem> {
    if cfg!(target_os = "linux") {
        Ok(v5::LoadModelLibrarySystem::Linux)
    } else if cfg!(target_os = "windows") {
        Ok(v5::LoadModelLibrarySystem::Windows)
    } else {
        Err(FrankaError::Model(
            "libfranka: Unsupported operating system!".to_string(),
        ))
    }
}

/// Downloads the robot's model library over `network` and builds a [`Model`].
///
/// Port of `franka::Model::Model(Network&)` (0.9.2), which is
/// `ModelLibrary(network)` and therefore `LibraryDownloader(network)` followed
/// by `LibraryLoader`.
///
/// # Temp-file lifecycle
///
/// The downloaded bytes are written to a uniquely named, mode `0600` file under
/// [`std::env::temp_dir`] (libfranka: `Poco::TemporaryFile::tempName()`), which
/// is `dlopen`ed and then **removed when the returned [`Model`] is dropped** —
/// the file guard lives inside the [`SoModelBackend`] the model owns. The guard
/// covers a normal drop and an unwind, but not `SIGINT`/`SIGTERM`,
/// `std::process::abort` or `std::process::exit`; a control loop killed with
/// Ctrl-C therefore leaves one ~330 KB file behind in the temp directory. Two
/// concurrent downloads (in the same process or in different ones) cannot
/// collide on the name.
///
/// # Platform limitation
///
/// The robot serves a *native* shared object, so this only works where the
/// robot has a build for the current `(architecture, system)` pair and where
/// the running process can load it. The request encodes whatever the host is
/// (`X64`/`X86`/`ARM64`/`ARM` and Linux/Windows, from `cfg!`), but in practice
/// only **x86-64 Linux**
/// works: the FER control unit ships `libfcimodels_x64.so`. Cross-compiled or
/// unusual targets get [`FrankaError::Model`] from the robot
/// (`"libfranka: Server reports error when loading model library."`) or from
/// `dlopen`.
///
/// # Errors
///
/// * [`FrankaError::InvalidOperation`] when `version` is not
///   [`FciVersion::V5`]: FCI v10 has no `LoadModelLibrary` command — an FR3
///   serves a URDF through `GetRobotModel` instead.
/// * [`FrankaError::Model`] when the target architecture or operating system
///   has no `LoadModelLibrary` encoding, when the robot answers
///   `Status::kError`, or when the shared object cannot be saved, loaded or
///   bound.
/// * [`FrankaError::Network`] / [`FrankaError::Protocol`] for the usual
///   command-socket failures.
#[cfg(feature = "model-library")]
pub fn load_from_robot(network: &Network, version: FciVersion) -> FrankaResult<Model> {
    let command = load_model_library_command(version)?;

    let request = v5::LoadModelLibraryRequest::new(architecture()?, system()?);
    let command_id = network.tcp.send_request(command, request.as_bytes())?;
    let message = network.tcp.blocking_receive_response(command_id)?;
    let response: v5::LoadModelLibraryResponse = parse_response(HeaderLayout::Robot, &message)?;

    // libfranka compares against `kSuccess` and reports one message for every other value,
    // including bytes that are not valid `Status` enumerators.
    if response.status != v5::LoadModelLibraryStatus::Success.to_u8() {
        return Err(FrankaError::Model(
            "libfranka: Server reports error when loading model library.".to_string(),
        ));
    }

    // The tail is everything after the 12-byte header and the status byte, i.e.
    // `header.size - 13` bytes; `message_payload` strips the header and the `[1..]` the
    // status. `parse_response` above has already established that the payload is non-empty.
    let library_bytes = &message_payload(HeaderLayout::Robot, &message)[1..];
    if library_bytes.is_empty() {
        return Err(FrankaError::Protocol(
            "libfranka: Incorrect TCP message size.".to_string(),
        ));
    }
    // SAFETY: `library_bytes` is the `libfcimodels` build the connected robot served over
    // the FCI command socket, and `dlopen`ing it executes its code. That is the trust
    // boundary libfranka 0.9.2 sits on (`LibraryDownloader` + `LibraryLoader`): the FCI
    // peer is already fully trusted, because it is the thing that commands the arm, and a
    // caller unwilling to extend that trust disables the `model-library` feature. See the
    // `# Security` section on `Robot::load_model`.
    unsafe { Model::from_model_library_bytes(library_bytes) }
}

/// [`load_from_robot`] as compiled **without** the `model-library` feature.
///
/// The signature is identical in both configurations so that callers
/// (`RobotImpl::load_model`) need no `cfg` of their own; the version check still
/// runs first, so an FCI v10 caller gets the same
/// [`FrankaError::InvalidOperation`] either way, and an FCI v5 caller gets a
/// [`FrankaError::Model`] explaining that this build cannot load one. No bytes
/// are requested from the robot, because there would be nothing to `dlopen`
/// them with.
#[cfg(not(feature = "model-library"))]
pub fn load_from_robot(_network: &Network, version: FciVersion) -> FrankaResult<Model> {
    load_model_library_command(version)?;
    Err(FrankaError::Model(
        "libfranka: this build of franka-rs was compiled without the `model-library` \
         feature, so the FCI v5 model library cannot be loaded."
            .to_string(),
    ))
}

#[cfg(feature = "model-library")]
impl Model {
    /// Builds a model from the bytes of a `libfcimodels` shared object.
    ///
    /// The offline twin of [`load_from_robot`]: same temp-file lifecycle (a
    /// `0600` file under [`std::env::temp_dir`], removed when the model is
    /// dropped), no network. Useful for a captured library and for tests.
    ///
    /// # Safety
    ///
    /// The bytes are written to a temporary file and `dlopen`ed, which executes
    /// the shared object's initialisers and, afterwards, its exported entry
    /// points in this process. The caller asserts that `bytes` are a **trusted**
    /// `libfcimodels` build for the current platform — served by a Franka
    /// control unit, or captured from one and kept where only trusted
    /// principals can write. See [`SoModelBackend::from_bytes`].
    ///
    /// # Errors
    ///
    /// [`FrankaError::Model`] when the file cannot be written, `dlopen`ed, or
    /// when any of the thirty `libfcimodels` symbols is missing.
    pub unsafe fn from_model_library_bytes(bytes: &[u8]) -> FrankaResult<Model> {
        // SAFETY: delegated to this function's own contract.
        let backend = unsafe { SoModelBackend::from_bytes(bytes)? };
        Ok(Model::from_backend(Box::new(backend)))
    }

    /// Builds a model from a `libfcimodels` shared object already on disk.
    ///
    /// Unlike [`Model::from_model_library_bytes`] the file is left alone when
    /// the model is dropped.
    ///
    /// # Safety
    ///
    /// The file is `dlopen`ed, which executes its code in this process. The
    /// caller asserts that `path` names a **trusted** `libfcimodels` build for
    /// the current platform and that it cannot be swapped before the load. See
    /// [`SoModelBackend::open`].
    ///
    /// # Errors
    ///
    /// [`FrankaError::Model`] when the file cannot be `dlopen`ed or when any of
    /// the thirty `libfcimodels` symbols is missing.
    pub unsafe fn from_model_library_path(path: &Path) -> FrankaResult<Model> {
        // SAFETY: delegated to this function's own contract.
        let backend = unsafe { SoModelBackend::open(path)? };
        Ok(Model::from_backend(Box::new(backend)))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn v10_has_no_load_model_library() {
        // A `Network` is not needed to reach the version check: it is the first thing
        // `load_from_robot` does, and `command_id` is a pure function of the version.
        assert!(command_id(FciVersion::V10, CommandKind::LoadModelLibrary).is_none());
        assert_eq!(
            command_id(FciVersion::V5, CommandKind::LoadModelLibrary),
            Some(13)
        );
    }

    #[test]
    fn the_unavailable_message_matches_the_agreed_wording() {
        let message = format!(
            "libfranka: {} is not available on FCI version {}.",
            CommandKind::LoadModelLibrary.name(),
            FciVersion::V10.number()
        );
        assert_eq!(
            message,
            "libfranka: Load Model Library is not available on FCI version 10."
        );
    }

    #[cfg(feature = "model-library")]
    #[test]
    fn this_build_asks_for_a_supported_platform() {
        // The CI and development target is x86-64 Linux, which is the only pair the FER
        // actually serves; the point of the assertion is that the mapping is wired up.
        if cfg!(all(target_arch = "x86_64", target_os = "linux")) {
            assert_eq!(
                architecture().unwrap(),
                v5::LoadModelLibraryArchitecture::X64
            );
            assert_eq!(system().unwrap(), v5::LoadModelLibrarySystem::Linux);
        }
    }
}
