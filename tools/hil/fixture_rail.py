"""Command-line inputs for a volatile rail calibration measured on the board under test."""


def add_rail_arguments(parser):
    parser.add_argument('--rail-avdd-uv', type=int, help='AVDD measured at the ADS supply, in microvolts.')
    parser.add_argument('--rail-avss-uv', type=int, help='AVSS measured at the ADS supply, in microvolts.')
    parser.add_argument('--rail-source-note', help='Measurement location, instrument or uncertainty.')


def rail_arguments(options):
    values = (options.rail_avdd_uv, options.rail_avss_uv, options.rail_source_note)
    if all(value is None for value in values):
        return []  # The collector verifies the existing external calibration.
    if (any(value is None for value in values) or not options.rail_source_note.strip()
            or not options.rail_avdd_uv > 0 > options.rail_avss_uv):
        raise ValueError('specify measured positive AVDD, negative AVSS and their source together')
    return ['--rail-avdd-uv', str(options.rail_avdd_uv), '--rail-avss-uv', str(options.rail_avss_uv),
            '--rail-source-note', options.rail_source_note]
