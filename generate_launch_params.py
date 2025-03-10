import os

def generate_launch_params(user_id, simulation_number):
    base_path = f"simulations/{user_id}/{user_id}_{simulation_number}"
    cad_path = os.path.join(base_path, "CADs")
    wind_path = os.path.join(base_path, "windsim")
    
    # Collect CAD models
    cad_files = [os.path.join(cad_path, f) for f in os.listdir(cad_path) if f.endswith('.stl')]
    
    # Collect wind simulation files
    wind_files = [os.path.join(wind_path, f) for f in os.listdir(wind_path) if f.endswith('.csv')]

    params = {
        'cad_files': cad_files,
        'wind_files': wind_files,
        'free_point': '0,0,0',  # Example value, modify as needed
        'cell_size': '0.1'  # Example value, modify as needed
    }
    
    return params

if __name__ == "__main__":
    import argparse
    import json
    
    parser = argparse.ArgumentParser(description='Generate launch parameters.')
    parser.add_argument('user_id', type=str, help='User ID')
    parser.add_argument('simulation_number', type=str, help='Simulation number')
    args = parser.parse_args()
    
    params = generate_launch_params(args.user_id, args.simulation_number)
    print(json.dumps(params))
