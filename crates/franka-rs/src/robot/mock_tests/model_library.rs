//! The `LoadModelLibrary` network path, which no simulator-free test other than these two
//! covers.

use super::*;

/// `LoadModelLibrary` goes out as a 2-byte `(architecture, system)` request under the v5
/// command id 13, and a `kError` status becomes the libfranka `ModelException` wording.
///
/// This is the only test of the `LoadModelLibrary` **network** path that runs without a
/// simulator; the sim test covers the happy path but cannot run in CI.
#[test]
#[cfg(feature = "model-library")]
fn load_model_library_reports_a_server_error_and_encodes_this_platform() {
    let server = MockServer::start_v5();
    let robot = server.connect(0);

    server.queue_response_for(
        CommandKind::LoadModelLibrary,
        &[v5::LoadModelLibraryStatus::Error.to_u8()],
    );
    match robot.load_model_v5() {
        Err(FrankaError::Model(message)) => assert_eq!(
            message,
            "libfranka: Server reports error when loading model library."
        ),
        other => panic!("expected a Model error, got {other:?}"),
    }

    assert_eq!(server.request_count_id(13), 1, "v5 LoadModelLibrary is 13");
    let payload = &server.payloads_for(CommandKind::LoadModelLibrary)[0];
    assert_eq!(payload.len(), 2, "LoadModelLibrary::Request is 2 bytes");
    if cfg!(all(target_arch = "x86_64", target_os = "linux")) {
        assert_eq!(
            payload.as_slice(),
            &[
                v5::LoadModelLibraryArchitecture::X64.to_u8(),
                v5::LoadModelLibrarySystem::Linux.to_u8(),
            ],
            "this build must ask for the x86-64 Linux library"
        );
    }
}

/// The library bytes are the response payload **after** the status byte: exactly one byte is
/// stripped, no more and no less.
///
/// A status-only payload therefore carries an empty library and is rejected as a protocol
/// error (if the client sliced from 0 it would try to `dlopen` the status byte instead), while
/// a payload with a tail reaches the loader (if the client sliced from 2 the four-byte tail
/// below would come out one byte short, but it would still reach `dlopen`; the empty-tail case
/// is what pins the boundary).
#[test]
#[cfg(feature = "model-library")]
fn load_model_library_strips_exactly_the_status_byte() {
    let server = MockServer::start_v5();
    let robot = server.connect(0);

    // Success, no tail: nothing to load.
    server.queue_response_for(
        CommandKind::LoadModelLibrary,
        &[v5::LoadModelLibraryStatus::Success.to_u8()],
    );
    match robot.load_model_v5() {
        Err(FrankaError::Protocol(message)) => {
            assert_eq!(message, "libfranka: Incorrect TCP message size.")
        }
        other => panic!("expected a Protocol error for an empty library, got {other:?}"),
    }

    // Success with a tail: the tail is written out and handed to `dlopen`, which rejects it
    // because it is not a shared object — proving the bytes got that far.
    let mut payload = vec![v5::LoadModelLibraryStatus::Success.to_u8()];
    payload.extend_from_slice(b"\x7fELF this is not a shared object");
    server.queue_response_for(CommandKind::LoadModelLibrary, &payload);
    match robot.load_model_v5() {
        Err(FrankaError::Model(message)) => assert!(
            message.starts_with("libfranka: Cannot load model library:"),
            "{message}"
        ),
        other => panic!("expected a Model error from dlopen, got {other:?}"),
    }

    assert_eq!(server.request_count_id(13), 2);
}
