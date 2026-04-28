# Rotation Methods Comparison

The path alignment node supports two methods for computing the Z-axis rotation between VIO and GPS paths.

## Least Squares (Default - RECOMMENDED)

**Method:** `rotation_method:='least_squares'`

### What It Does
Finds the rotation angle θ that minimizes the weighted sum of squared errors:
```
minimize: Σ w[i] × ||R(θ) × vio_vector[i] - gps_vector[i]||²
```

Where:
- R(θ) is the Z-axis rotation matrix
- w[i] = gps_distance[i] × vio_distance[i] (weight)

### Closed-Form Solution
For 2D rotation around Z-axis, the optimal angle is:
```
θ = atan2(Σ w[i]×b[i], Σ w[i]×a[i])

where:
  a[i] = vio_x × gps_x + vio_y × gps_y  (dot product in XY)
  b[i] = vio_x × gps_y - vio_y × gps_x  (cross product z-component)
```

### Advantages
✅ **Mathematically optimal** - Provably minimizes alignment error
✅ **More robust** - Better handles outliers and noisy data
✅ **Industry standard** - Used in ICP and similar alignment algorithms
✅ **Direct alignment** - Minimizes actual vector differences
✅ **No intermediate angles** - Works directly with vector components

### When to Use
- **Always** (it's the default for good reason)
- Especially when data is noisy
- When you want the most accurate alignment
- When vectors don't all point in similar directions

### Example
```bash
# Default (recommended)
ros2 launch path_alignment path_alignment_launch.py

# Explicit
ros2 launch path_alignment path_alignment_launch.py rotation_method:=least_squares
```

---

## Circular Mean (Legacy)

**Method:** `rotation_method:='circular_mean'`

### What It Does
1. Computes weighted average heading for GPS vectors
2. Computes weighted average heading for VIO vectors  
3. Rotation = difference between the two average headings

```
gps_mean = atan2(Σ w×sin(θ_gps), Σ w×cos(θ_gps))
vio_mean = atan2(Σ w×sin(θ_vio), Σ w×cos(θ_vio))
rotation = gps_mean - vio_mean
```

### Advantages
✅ **Intuitive** - Easy to understand conceptually
✅ **Fast** - Slightly faster computation (negligible difference)
✅ **Good for parallel paths** - Works well when all vectors point similar directions

### Disadvantages
❌ **Not optimal** - Doesn't minimize alignment error
❌ **Two-step approximation** - Computes headings first, then difference
❌ **Can be skewed** - Outliers in one direction can dominate
❌ **Assumes consistency** - Assumes one "average direction" represents whole path

### When to Use
- For comparison with old results
- When debugging
- If you have a specific reason to prefer it

### Example
```bash
ros2 launch path_alignment path_alignment_launch.py rotation_method:=circular_mean
```

---

## Comparison Example

Given the same GPS and VIO vectors:

### Least Squares:
```
Computing optimal rotation that minimizes total error...
For each vector pair:
  - Computes dot product: a[i] = vio·gps (alignment measure)
  - Computes cross product: b[i] = vio×gps (rotation measure)
  - Weights by movement magnitude
Result: θ = 45.18° (provably minimizes Σ||R(θ)×vio - gps||²)
```

### Circular Mean:
```
Computing average directions...
GPS vectors point at: 45°, 47°, 43°, 46°, 44° (varying directions)
→ GPS mean heading: 45.0°

VIO vectors point at: 0°, 2°, -2°, 1°, -1° (varying directions)  
→ VIO mean heading: 0.0°

Rotation = 45.0° - 0.0° = 45.0°
```

In this simple case they're similar, but with outliers or inconsistent directions, least squares will give better results.

---

## Theoretical Background

### Least Squares Derivation

We want to find θ that minimizes:
```
E(θ) = Σ w[i] × ||R(θ) × v[i] - g[i]||²
```

Taking the derivative with respect to θ and setting to zero:
```
dE/dθ = Σ w[i] × 2(R(θ)×v[i] - g[i]) · dR/dθ×v[i] = 0

For 2D rotation:
R(θ) = [cos(θ)  -sin(θ)]
       [sin(θ)   cos(θ)]

dR/dθ = [-sin(θ)  -cos(θ)]
        [ cos(θ)  -sin(θ)]

After algebraic manipulation:
→ θ = atan2(Σ w×(vx×gy - vy×gx), Σ w×(vx×gx + vy×gy))
```

This is a **closed-form solution** - no iteration needed!

### Circular Mean Rationale

The circular mean uses the **directional statistics** approach:
- Treats each vector as a direction (angle)
- Computes average direction using von Mises distribution
- Assumes rotation is the difference between average directions

This is **not** the same as minimizing alignment error, but works well when all vectors are consistent.

---

## Recommendation

**Use `least_squares` (the default).** It's:
- Mathematically sound
- More robust
- Actually optimal
- Just as fast
- The industry standard approach

Only use `circular_mean` if you have old results to compare against or are debugging.

---

## Performance Notes

Both methods have O(n) complexity where n is the number of vector pairs. The difference in computation time is negligible (< 1ms for typical path lengths).

The main difference is **accuracy**, not speed.
