import math

class Conversions:
    @staticmethod
    def rotor_rotations_to_wheel_rotations(
        rotor_rotations: float,
        gear_ratio: float
    ) -> float:
        """Convert motor rotor rotations to wheel rotations."""
        return rotor_rotations / gear_ratio

    @staticmethod
    def wheel_rotations_to_meters(
        wheel_rotations: float,
        wheel_radius_m: float
    ) -> float:
        """Convert wheel rotations to linear travel in meters."""
        return wheel_rotations * (2.0 * math.pi * wheel_radius_m)

    @staticmethod
    def rotor_rps_to_wheel_mps(
        rotor_rps: float,
        gear_ratio: float,
        wheel_radius_m: float
    ) -> float:
        """Convert motor rotor rotations/sec to wheel meters/sec."""
        wheel_rps = rotor_rps / gear_ratio
        return wheel_rps * (2.0 * math.pi * wheel_radius_m)
    
    @staticmethod
    def rotations_to_angle_rad(
        rotations: float
    ) -> float:
        """Convert rotations to radians."""
        return rotations * (2.0 * math.pi)