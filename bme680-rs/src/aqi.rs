#![forbid(unsafe_code)]

pub fn air_quality_index(humidity: u32, gas_resist: u32) -> u32 {
    let humidity_ref = 40;
    let gas_lower_limit = 5000;
    let gas_upper_limit = 50000;

    let humidity_score = if (38000..=42000).contains(&humidity) {
        25000
    } else if humidity < 38000 {
        humidity * 25 / humidity_ref
    } else {
        (4166660 - 41666 * humidity_ref - 25 * humidity) / (100 - humidity_ref)
    };

    let gas_resist = gas_resist.clamp(gas_lower_limit, gas_upper_limit);
    let gas_score = (gas_resist - gas_lower_limit) * 75000 / (gas_upper_limit - gas_lower_limit);

    humidity_score + gas_score
}

#[cfg(test)]
mod test {
    use crate::air_quality_index;

    #[test]
    fn air_quality_index_test() {
        let tests = [
            (air_quality_index(30000, 1000), 18750),
            (air_quality_index(40000, 1000), 25000),
            (air_quality_index(50000, 1000), 20833),
            (air_quality_index(30000, 10000), 27083),
            (air_quality_index(40000, 10000), 33333),
            (air_quality_index(50000, 10000), 29166),
            (air_quality_index(30000, 100000), 93750),
            (air_quality_index(40000, 100000), 100000),
            (air_quality_index(50000, 100000), 95833),
        ];

        for (value, expected) in tests {
            assert_eq!(value.abs_diff(expected), 0);
        }
    }
}
