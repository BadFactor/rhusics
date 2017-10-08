//! Physics related functionality
//!
#![allow(missing_docs)]

use cgmath::prelude::*;

use {BodyPose, Real};

#[derive(Debug)]
pub enum Force<V, R> {
    Linear(V),
    Angular(R),
}

#[derive(Debug)]
pub struct Particle<V, R> {
    mass: Real,
    inverse_mass: Real,
    forces: Vec<Force<V, R>>,
}

#[derive(Debug)]
pub struct Velocity<V, R> {
    linear: V,
    angular: R,
}

#[derive(Debug)]
pub struct NextFrame<T, V, R> {
    transform: T,
    velocity: Velocity<V, R>
}

#[derive(Debug)]
pub struct RigidBody<V, R, I> {
    mass: Real,
    inverse_mass: Real,
    inertia_tensor: I,
    inverse_inertia_tensor: I,
    forces: Vec<Force<V, R>>
}

/// Trait bound used for transforms throughout the physics part of the library
pub trait Pose<P>: Transform<P>
where
    P: EuclideanSpace<Scalar = Real>,
{
    /// The rotational data type used by the concrete implementation
    type Rotation: Rotation<P>;

    /// Borrows the position attribute
    fn position(&self) -> &P;

    /// Borrows the rotation attribute
    fn rotation(&self) -> &Self::Rotation;

    /// Sets the position attribute
    fn set_position(&mut self, pos: P);

    /// Sets the rotation attribute
    fn set_rotation(&mut self, rot: Self::Rotation);

    /// Checks to see if the transform is dirty. Used by the collision system to see if bounds need
    /// to be recalculated.
    fn dirty(&self) -> bool;

    /// Borrows the inverse rotation attribute
    fn inverse_rotation(&self) -> &Self::Rotation;
}

impl<P, R> Pose<P> for BodyPose<P, R>
where
    R: Rotation<P>,
    P: EuclideanSpace<Scalar = Real>,
{
    type Rotation = R;

    fn position(&self) -> &P {
        &self.position
    }

    fn rotation(&self) -> &Self::Rotation {
        &self.rotation
    }

    fn set_position(&mut self, pos: P) {
        self.position = pos;
    }

    fn set_rotation(&mut self, rot: Self::Rotation) {
        self.rotation = rot;
    }

    fn dirty(&self) -> bool {
        self.dirty
    }

    fn inverse_rotation(&self) -> &Self::Rotation {
        &self.inverse_rotation
    }
}
