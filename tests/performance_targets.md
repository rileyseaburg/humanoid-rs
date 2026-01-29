# Performance Targets

This document defines target performance metrics and regression thresholds for the humanoid-rs framework.

## Control Loop Performance

### Latency Requirements

| Component | Target Latency | Maximum Acceptable | Critical? |
|-----------|---------------|-------------------|-----------|
| Control Loop Update | < 500 µs | 1 ms | Yes |
| Joint Position Read | < 50 µs | 100 µs | Yes |
| Joint Command Write | < 50 µs | 100 µs | Yes |
| Trajectory Interpolation | < 100 µs | 500 µs | No |
| IK Solve (single target) | < 1 ms | 5 ms | No |
| MPC Solve | < 5 ms | 20 ms | Yes |

### Frequency Requirements

| Loop Type | Target Frequency | Minimum Frequency | Notes |
|-----------|-----------------|-------------------|-------|
| Main Control Loop | 2 kHz | 1 kHz | Critical for stability |
| Sensor Reading | 2 kHz | 1 kHz | IMU, joint encoders |
| Trajectory Planning | 100 Hz | 50 Hz | Path generation |
| IK Updates | 100 Hz | 50 Hz | End-effector control |
| MPC Horizon | 50 Hz | 20 Hz | Long-term planning |
| Visualization | 60 Hz | 30 Hz | Non-critical |

## Physics Simulation Performance

### Step Time Targets

| Configuration | Target | Maximum | Notes |
|--------------|--------|---------|-------|
| 29 joints, 1ms step | 500 µs | 1 ms | Full humanoid |
| 20 joints, 1ms step | 300 µs | 600 µs | Reduced DOF |
| 10 joints, 1ms step | 150 µs | 300 µs | Simplified model |

### Collision Detection

| Scenario | Target | Maximum |
|----------|--------|---------|
| Self-collision (single) | < 50 µs | 100 µs |
| Self-collision (all pairs) | < 500 µs | 1 ms |
| Environment collision | < 100 µs | 200 µs |

### Sensor Update Overhead

| Sensor Type | Update Time | Max Overhead |
|-------------|-------------|--------------|
| Joint encoders | < 10 µs | 20 µs |
| IMU | < 20 µs | 50 µs |
| Force/Torque sensors | < 30 µs | 60 µs |
| Camera (simulated) | < 5 ms | 10 ms |

## Memory Usage Limits

### Runtime Memory

| Component | Target | Maximum | Notes |
|-----------|--------|---------|-------|
| Control Loop (steady state) | < 10 MB | 50 MB | Heap only |
| Physics Simulation | < 100 MB | 500 MB | Including geometry |
| IK Solver | < 5 MB | 20 MB | Per solver instance |
| MPC Controller | < 50 MB | 200 MB | Prediction matrices |
| Total System | < 500 MB | 1 GB | All components |

### Memory Allocation Patterns

- **Zero-allocation paths**: Control loop, sensor reading, and emergency stop paths must not allocate during steady-state operation
- **Pre-allocated buffers**: Trajectory buffers, joint state vectors, and IK workspace should be pre-allocated
- **Allocation limits**: Non-critical paths may allocate but should use object pools where possible

## IK Solver Performance

### Convergence Metrics

| Chain Length | Max Iterations | Target Time | Max Time |
|-------------|----------------|-------------|----------|
| 2 DOF (analytic) | 1 | < 10 µs | 50 µs |
| 3 DOF | 20 | < 200 µs | 500 µs |
| 5 DOF | 50 | < 500 µs | 2 ms |
| 7 DOF | 100 | < 1 ms | 5 ms |
| 10 DOF | 100 | < 2 ms | 10 ms |

### Accuracy Requirements

| Metric | Target | Acceptable |
|--------|--------|------------|
| Position error | < 1 mm | < 5 mm |
| Orientation error | < 1° | < 5° |
| Joint limit satisfaction | 100% | > 95% |
| Success rate (reachable) | > 99% | > 95% |

## MPC Controller Performance

### Computation Time

| Horizon Length | QP Solve Time | Total Time |
|----------------|---------------|------------|
| 10 steps | < 2 ms | < 3 ms |
| 20 steps | < 5 ms | < 7 ms |
| 50 steps | < 20 ms | < 25 ms |

### Prediction Accuracy

| Time Horizon | CoM Position Error | ZMP Tracking Error |
|--------------|-------------------|-------------------|
| 0.5 s | < 5 mm | < 2 mm |
| 1.0 s | < 20 mm | < 10 mm |
| 2.0 s | < 50 mm | < 30 mm |

## Regression Thresholds

### Automatic Regression Detection

| Metric | Warning Threshold | Error Threshold |
|--------|------------------|----------------|
| Control loop latency | +20% | +50% |
| Physics step time | +20% | +50% |
| IK solve time | +30% | +100% |
| Memory usage | +10% | +50% |
| Allocation rate (critical paths) | Any increase | Any increase |

### Benchmark Comparisons

Performance benchmarks should maintain:
- No more than 5% regression in average latency
- No more than 10% regression in p99 latency
- No more than 5% regression in throughput
- No allocation in previously zero-allocation paths

## CI Performance Checks

### Required Checks

1. **Unit Tests**: Must complete in < 30 seconds
2. **Integration Tests**: Must complete in < 5 minutes
3. **Benchmarks**: Must run and report metrics
4. **Memory Leaks**: No leaks detected in valgrind/address sanitizer
5. **Determinism**: Golden file tests must pass

### Benchmark Baselines

Benchmark baselines are stored in `benches/baselines/` and updated:
- Automatically on release branches
- Manually via `cargo bench --save-baseline`
- Never on feature branches without approval

## Platform-Specific Targets

### Development Machine (Reference)
- CPU: Intel i7 or AMD Ryzen 7
- RAM: 16 GB
- Target: All metrics as specified above

### Target Hardware (Unitree G1)
- CPU: ARM-based (Jetson or similar)
- RAM: 8 GB
- Adjustment: 2x latency targets acceptable

### Real-time Constraints

For hard real-time deployment:
- Worst-case execution time (WCET) must be < 70% of deadline
- All memory must be pre-allocated
- No blocking operations in control loop
- Priority inheritance for mutexes

## Monitoring and Alerting

### Metrics to Track

1. Control loop jitter (max deviation from target period)
2. Missed deadlines (count per minute)
3. Memory growth over time (MB/hour)
4. IK success rate (%)
5. Physics solver iterations per step

### Alert Thresholds

- **Warning**: Jitter > 10% of period
- **Critical**: Missed deadlines > 1%
- **Emergency**: Determinism test failures

## Optimization Priorities

### P0 (Critical)
- Control loop latency at 1-2 kHz
- Zero-allocation critical paths
- Emergency stop latency < 1 ms

### P1 (High)
- Physics step at 500-1000 Hz
- IK solve < 5 ms
- Memory usage < 500 MB

### P2 (Medium)
- Visualization at 60 Hz
- Trajectory planning < 10 ms
- MPC solve < 20 ms

### P3 (Low)
- Non-real-time tools
- Development features
- Visualization quality

## Measurement Methodology

### Timing Measurements

```rust
// Use Criterion for statistical benchmarking
use criterion::{black_box, Criterion};

fn benchmark_example(c: &mut Criterion) {
    c.bench_function("operation", |b| {
        b.iter(|| {
            black_box(expensive_operation());
        });
    });
}
```

### Memory Measurements

- Use `dhat` for heap profiling
- Use `/usr/bin/time -v` for peak RSS
- Track allocation counts in benchmarks
- Monitor with `valgrind --tool=massif` for detailed analysis

### Statistical Requirements

- Minimum 100 samples per benchmark
- Report: mean, median, p90, p99, stddev
- Detect and handle outliers (> 3 sigma)
- Use warmup iterations (minimum 3)

## Version History

| Version | Date | Changes |
|---------|------|---------|
| 0.1.0 | 2024-01-28 | Initial targets defined |
| 0.2.0 | TBD | Add real-time constraints |
| 0.3.0 | TBD | Add embedded targets |

## Notes

- All timing measurements are wall-clock time
- Memory measurements exclude OS overhead
- Targets assume release mode (`--release`)
- Debug builds may be 10-100x slower
