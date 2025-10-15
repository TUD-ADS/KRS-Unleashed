from configuration_manager import VitisConfigManager

if __name__ == "__main__":
    # Example file paths
    config_file = "scripts/config.yaml"

    manager = VitisConfigManager(config_file)
    if manager.load_and_validate():
        print("✅ Configuration is valid!")
        # Example access
        print("Project name:", manager.get("Project.name"))
        print("First kernel name:", manager.get("Project.kernels.0.name"))
    else:
        print("❌ Configuration errors:")
        for err in manager.get_errors():
            print("  -", err)
