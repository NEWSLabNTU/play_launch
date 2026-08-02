// Performance tests

use play_launch_parser::parse_launch_file;
use std::{collections::HashMap, path::PathBuf};

/// Helper to get fixture path from crate tests directory
fn get_fixture_path(filename: &str) -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("tests/fixtures/launch")
        .join(filename)
}

#[test]
fn test_performance_simple_launch() {
    // Benchmark parsing performance on a simple launch file
    use std::time::Instant;

    let fixture = get_fixture_path("test_args.launch.xml");
    let args = HashMap::new();

    let start = Instant::now();
    let iterations = 100;

    for _ in 0..iterations {
        let result = parse_launch_file(&fixture, args.clone());
        assert!(result.is_ok());
    }

    let duration = start.elapsed();
    let avg_ms = duration.as_millis() as f64 / iterations as f64;

    println!("\nPerformance - Simple Launch File:");
    println!("  Total time ({} iterations): {:?}", iterations, duration);
    println!("  Average per parse: {:.2}ms", avg_ms);
    println!("  Throughput: {:.0} parses/sec", 1000.0 / avg_ms);

    // Assert reasonable performance (should be well under 100ms for simple files)
    assert!(
        avg_ms < 10.0,
        "Simple parse should be < 10ms, got {:.2}ms",
        avg_ms
    );
}

#[test]
fn test_performance_complex_nested() {
    // Benchmark parsing performance on a complex nested launch file
    use std::time::Instant;

    let fixture = get_fixture_path("test_complex_nested.launch.xml");
    let args = HashMap::new();

    let start = Instant::now();
    let iterations = 50;

    for _ in 0..iterations {
        let result = parse_launch_file(&fixture, args.clone());
        assert!(result.is_ok());
    }

    let duration = start.elapsed();
    let avg_ms = duration.as_millis() as f64 / iterations as f64;

    println!("\nPerformance - Complex Nested Launch File:");
    println!("  Total time ({} iterations): {:?}", iterations, duration);
    println!("  Average per parse: {:.2}ms", avg_ms);
    println!("  Throughput: {:.0} parses/sec", 1000.0 / avg_ms);

    // Assert reasonable performance (complex files should still be fast)
    assert!(
        avg_ms < 50.0,
        "Complex parse should be < 50ms, got {:.2}ms",
        avg_ms
    );
}

#[test]
fn test_performance_with_includes() {
    // Benchmark parsing performance with includes
    use std::time::Instant;

    let fixture = get_fixture_path("test_all_features.launch.xml");
    let args = HashMap::new();

    let start = Instant::now();
    let iterations = 50;

    for _ in 0..iterations {
        let result = parse_launch_file(&fixture, args.clone());
        assert!(result.is_ok());
    }

    let duration = start.elapsed();
    let avg_ms = duration.as_millis() as f64 / iterations as f64;

    println!("\nPerformance - Launch File with Includes:");
    println!("  Total time ({} iterations): {:?}", iterations, duration);
    println!("  Average per parse: {:.2}ms", avg_ms);
    println!("  Throughput: {:.0} parses/sec", 1000.0 / avg_ms);

    // Assert reasonable performance
    assert!(
        avg_ms < 50.0,
        "Parse with includes should be < 50ms, got {:.2}ms",
        avg_ms
    );
}
