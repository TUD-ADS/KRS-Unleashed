from simple_yaml_parser import SimpleYAMLParser
from constants import Constants

if __name__ == "__main__":
    # Example file paths
    config_file = "scripts/config.yaml"

    parser = SimpleYAMLParser()
    result = parser.parse_file(config_file)

    print("\nParsed result:")
    print(result)
    print("Project name:", result.get("Project").get("name"))
    print("First kernel name:", result.get(
        "Project").get("kernels")[0].get('name'))
    const_file = Constants(result)
    print(const_file.get("hls_clock_target"))
