//! Vehicle classes, and the sets of them a lane allows (see "Vehicle classes"
//! in `docs/DESIGN.md`), as SUMO's vehicle classes and lane permissions are.
//! A class only restricts which lanes a vehicle may use: the searches, merges
//! and junctions see every vehicle on a lane, allowed or not.

/// What kind of vehicle it is, for which lanes it may use.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum VehicleClass {
    #[default]
    Car,
    Truck,
    Bus,
    Tram,
}

impl VehicleClass {
    /// Every class, in the order of their bits in a [`Classes`].
    pub const ALL: [VehicleClass; 4] = [
        VehicleClass::Car,
        VehicleClass::Truck,
        VehicleClass::Bus,
        VehicleClass::Tram,
    ];

    /// Its place in [`VehicleClass::ALL`].
    pub const fn index(self) -> usize {
        self as usize
    }
}

/// A set of vehicle classes, as the classes a lane allows: a bitmask.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Classes(u8);

impl Classes {
    /// Every class.
    pub const ALL: Classes = Classes(0b1111);
    /// Every class but trams: a lane's by default.
    pub const ALL_BUT_TRAMS: Classes = Classes(0b0111);
    /// Trams alone: a tram track of its own.
    pub const TRAMS: Classes = Classes(1 << VehicleClass::Tram as u8);

    /// Just `class`.
    pub const fn only(class: VehicleClass) -> Classes {
        Classes(1 << class as u8)
    }

    /// Whether it holds `class`.
    pub const fn contains(self, class: VehicleClass) -> bool {
        self.0 & (1 << class as u8) != 0
    }

    /// The classes both hold.
    pub const fn and(self, other: Classes) -> Classes {
        Classes(self.0 & other.0)
    }

    /// The classes either holds.
    pub const fn or(self, other: Classes) -> Classes {
        Classes(self.0 | other.0)
    }

    /// Whether it holds no class.
    pub const fn is_empty(self) -> bool {
        self.0 == 0
    }
}

impl Default for Classes {
    fn default() -> Self {
        Classes::ALL_BUT_TRAMS
    }
}
