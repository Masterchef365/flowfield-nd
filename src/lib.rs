use std::ops::{Index, IndexMut};

use ndarray::{Array, Array2, ArrayView, IxDyn};

#[derive(Clone)]
pub struct SolverConfig {
    pub dt: f32,
    pub n_iters: usize,
}

#[derive(Clone)]
pub struct FlowField {
    /// One single-component flow field for each face of the N-dimensional cube.
    flow: Vec<Array<f32, IxDyn>>,
    width: usize,
}

pub struct FluidSolver {
    flow: FlowField,
}

/// This is a two-dimensional array for the point cloud, where the last dimension corresponds
/// to the coponent of the vector, and the second dimension corresponds to which point is selected
#[derive(Clone)]
pub struct PointCloud(pub Array2<f32>);

impl FluidSolver {
    pub fn new(flow: FlowField) -> Self {
        Self { flow }
    }

    pub fn step(&mut self, config: &SolverConfig) {
        self.jacobi(config.n_iters);
        self.advect(config.dt);
    }

    fn jacobi(&mut self, n_iters: usize) {
        for i in 0..n_iters * 2 {
            self.jacobi_half_step(i & 1 != 0);
        }
    }

    fn jacobi_half_step(&mut self, parity: bool) {
        let shape_minus_one = vec![self.width() - 2; self.dims()];
        for (idx, tl) in fill_shape(&shape_minus_one).enumerate() {
            if (idx & 1 == 0) != parity {
                continue;
            }

            for dim in 0..self.dims() {
                let mut other = tl.clone();
                other[dim] += 1;

            }

        }
    }

    fn advect(&mut self, dt: f32) {
    }

    pub fn shape(&self) -> Vec<usize> {
        self.flow.shape()
    }

    pub fn get_flow(&self) -> &FlowField {
        &self.flow
    }

    pub fn get_flow_mut(&mut self) -> &mut FlowField {
        &mut self.flow
    }

    /// Number of dimensions
    pub fn dims(&self) -> usize {
        self.flow.dims()
    }

    pub fn width(&self) -> usize {
        self.flow.width()
    }
}

/// Sweeps the given point cloud along the given flow field
pub fn sweep_pointcloud(pcld: &mut PointCloud, flow: &FlowField, dt: f32) {
    for mut pos in pcld.0.outer_iter_mut() {
        if let Some(vel) = flow.n_linear_interp(pos.as_slice().unwrap(), Boundary::Zero) {
            pos.iter_mut().zip(vel).for_each(|(p, v)| *p += v * dt);
        } else {
            // TODO: What happens when we go out of bounds?
        }
    }
}

impl FlowField {
    pub fn new(dimensions: usize, width: usize) -> Self {
        let mut flow = vec![];
        for dim in 0..dimensions {
            // We need an extra array element in each direction we're not principally responsible
            // for (as a cell)
            let mut shape = vec![width + 1; dimensions];
            shape[dim] -= 1;

            flow.push(Array::zeros(shape));
        }

        Self {
            flow,
            width,
        }
    }

    pub fn shape(&self) -> Vec<usize> {
        vec![self.width; self.dims()]
    }

    pub fn width(&self) -> usize {
        self.width
    }

    pub fn dims(&self) -> usize {
        self.flow.len()
    }

    pub fn get_axes(&self) -> &[Array<f32, IxDyn>] {
        &self.flow
    }

    pub fn get_axes_mut(&mut self) -> &mut [Array<f32, IxDyn>] {
        &mut self.flow
    }

    /*
    /// Returns an iterator over the cell positions (as integers)
    pub fn enumerate(&self) -> impl Iterator<Item = Vec<usize>> + 'static {
        enumerate_coords(self.width(), self.dims())
    }
    */

    /// linear interpolation of the flow map at a point in ND space
    pub fn n_linear_interp(&self, position: &[f32], boundary: Boundary) -> Option<Vec<f32>> {
        assert_eq!(position.len(), self.dims());

        let mut output = vec![0.0; self.dims()];

        // Iterate over x, y, z, ...
        for (coord_idx, flow_channel) in self.flow.iter().enumerate() {
            let mut pos_off = position.to_vec();
            // Calculate dimensional offset
            for (i, pos) in pos_off.iter_mut().enumerate() {
                // Offset inside this sub-grid. 
                // The reasoning for this is that if we are a face of one dimension on the grid, 
                // then the zero vale of our dimension should also be zero in the real world.
                // Everything else is offset by 1/2 of a square because setting that one zero
                // centered the face there!

                // Staggered grid
                if i != coord_idx {
                    *pos += 0.5;
                }
            }
            output[coord_idx] = n_linear_interp_array(flow_channel, &pos_off, boundary)?;
        }

        Some(output)
    }
}

fn lerp(a: f32, b: f32, t: f32) -> f32 {
    (1. - t) * a + t * b
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Boundary {
    Nearest,
    Zero,
}

pub fn n_linear_interp_array(
    arr: &Array<f32, IxDyn>,
    input_pos: &[f32],
    boundary: Boundary,
) -> Option<f32> {
    let dims = arr.shape().len();
    assert_eq!(input_pos.len(), dims);

    let fractional: Vec<f32> = input_pos.iter().map(|p| p.fract() as f32).collect();
    let upper_left: Vec<i32> = input_pos.iter().map(|p| p.floor() as i32).collect();

    // Iterate over neighbors in n-space.
    // In 1D this is nearest neighbor
    // in 2D these are the vertices of the square
    // in 3D these are ditto cube
    // ... hypercubes
    let mut neighborhood = Array::from_elem(vec![2; dims], 0.0);

    for combo in combos(0, 1, 1, dims) {
        // Offset position, and do bounds check
        let pos = upper_left
            .iter()
            .zip(&combo)
            .map(|(p, c)| p + c)
            .enumerate()
            .map(|(idx, pos)| boundary.clamp_or_none(pos, arr.shape()[idx]))
            .collect::<Option<Vec<usize>>>()?;
        let val = arr[pos.as_slice()];

        let combo_usize = combo
            .into_iter()
            .map(|c| c as usize)
            .collect::<Vec<usize>>();

        *neighborhood.index_mut(combo_usize.as_slice()) = val;
    }

    let neighborhood_lerp = neighborhood.as_slice_memory_order_mut().unwrap();

    // Fast linear interpolate
    for dimension in 0..dims {
        let stride = 1 << dimension;
        for slice in neighborhood_lerp.chunks_exact_mut(stride * 2) {
            slice[0] = lerp(slice[0], slice[stride], fractional[dims - dimension - 1]);
        }
    }

    Some(neighborhood_lerp[0])
}

impl Boundary {
    pub fn clamp_or_none(&self, coord: i32, width: usize) -> Option<usize> {
        match self {
            Self::Nearest => Some(coord.max(width as i32 - 1) as usize),
            Self::Zero => (0..width)
                .contains(&coord.try_into().ok()?)
                .then(|| coord as usize),
        }
    }
}

impl PointCloud {
    fn new(dimensions: usize) -> Self {
        Self(Array::zeros((dimensions, 0)))
    }
}

/*
pub fn enumerate_coords(width: usize, n_dims: usize) -> impl Iterator<Item = Vec<usize>> + 'static {
    let mut coordinate = vec![0; n_dims];
    std::iter::from_fn(move || {
        let inc = coordinate.iter().position(|c| c + 1 != width)?;
        let cpy = coordinate.clone();
        coordinate[inc] += 1;
        coordinate[0..inc].fill(0);
        Some(cpy)
    })
    .chain(std::iter::once(vec![width - 1; n_dims]))
}
*/

pub fn neighborhood(n_dims: usize) -> impl Iterator<Item=Vec<i32>> {
    combos(-1, 1, 1, n_dims)
}

pub fn combos(min: i32, max: i32, step: i32, n_dims: usize) -> impl Iterator<Item=Vec<i32>> {
    let mut dims = vec![min; n_dims];
    std::iter::from_fn(move || {
        // Quitting
        if dims.is_empty() {
            return None;
        }
        
        let ret = dims.clone();
       
        // Cascading add
        for i in 0..n_dims {
            if dims[i] < max {
                dims[i] += step;
                break;
            } else {
                dims[i] = min;
            }
        }
         
        // Remember to quit
        if ret.iter().all(|&v| v == max) {
            dims = vec![];
        }
        
        Some(ret)
    })
}

pub fn fill_shape(shape: &[usize]) -> impl Iterator<Item=Vec<usize>> + '_ {
    let mut dims = vec![0; shape.len()];

    std::iter::from_fn(move || {
        // Quitting
        if dims.is_empty() {
            return None;
        }
        
        let ret = dims.clone();
       
        // Cascading add
        for i in 0..shape.len() {
            if dims[i] + 1 < shape[i] {
                dims[i] += 1;
                break;
            } else {
                dims[i] = 0;
            }
        }
         
        // Remember to quit
        if ret.iter().enumerate().all(|(idx, &v)| v + 1 == shape[idx]) {
            dims = vec![];
        }
        
        Some(ret)
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn interp_test() {
        let mut arr = Array::from_elem(vec![2], 0.0);
        arr[[1]] = 1.0;

        assert_eq!(n_linear_interp_array(&arr, &[0.46], Boundary::Zero).unwrap(), 0.46);
    }

    #[test]
    fn interp_test_2() {
        let mut arr = Array::from_elem(vec![2; 2], 0.0);
        arr[[1, 0]] = 1.0;
        arr[[1, 1]] = 10.0;

        assert_eq!(n_linear_interp_array(&arr, &[0.5, 0.5], Boundary::Zero).unwrap(), 2.75);
        assert_eq!(n_linear_interp_array(&arr, &[0.0, 0.5], Boundary::Zero).unwrap(), 0.);
        assert!((n_linear_interp_array(&arr, &[0.9999, 0.5], Boundary::Zero).unwrap() - 11.0/2.0).abs() < 1e-3);
    }
}

impl Default for SolverConfig {
    fn default() -> Self {
        Self { dt: 0.1 }
    }
}
