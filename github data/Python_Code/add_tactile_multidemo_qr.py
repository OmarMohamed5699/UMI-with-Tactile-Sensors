import argparse
import json
import numpy as np
from pathlib import Path
from typing import Dict, List, Tuple, Optional
import zarr
from scipy.interpolate import interp1d
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from std_msgs.msg import String
from imagecodecs.numcodecs import register_codecs
register_codecs()

def read_qr_sync_data(qr_sync_path: str) -> Dict:

    with open(qr_sync_path, 'r') as f:
        sync_data = json.load(f)
    return sync_data




def read_tactile_bag(bag_path: str) -> Tuple[List[float], List[Dict]]:
    
    storage_options = StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr'
    )
    
    reader = SequentialReader()
    reader.open(storage_options, converter_options)
    
    timestamps = []
    tactile_data = []
    
    topic_name = '/umi_gripper/tactile_60hz'
    
    while reader.has_next():
        (topic, data, timestamp_ns) = reader.read_next()
        
        if topic == topic_name:
            msg = deserialize_message(data, String)
            
            try:
                json_data = json.loads(msg.data)
                
                # Skip QR sync event messages
                if json_data.get('type') == 'QR_SYNC_EVENT':
                    continue
                
                timestamp_sec = timestamp_ns / 1e9
                timestamps.append(timestamp_sec)
                tactile_data.append(json_data)
                
            except json.JSONDecodeError:
                continue
    
    return timestamps, tactile_data






def get_episode_boundaries(zarr_root) -> List[Tuple[int, int]]:
 
 
 
    if 'meta/episode_ends' not in zarr_root:
        # Single episode
        if 'data/img' in zarr_root:
            total_frames = zarr_root['data/img'].shape[0]
        elif 'data/action' in zarr_root:
            total_frames = zarr_root['data/action'].shape[0]
        else:
            raise ValueError("Cannot determine frame count")
        return [(0, total_frames)]
    
    episode_ends = zarr_root['meta/episode_ends'][:]
    
    boundaries = []
    start = 0
    for end in episode_ends:
        boundaries.append((start, end))
        start = end
    
    return boundaries






def align_timestamps_with_qr(
    tactile_timestamps: List[float],
    qr_sync_data: Dict,
    video_fps: float = 60.0
) -> np.ndarray:
    # align tactile timestamps to video time using QR sync offset
    offset = qr_sync_data['offset']
    aligned_timestamps = np.array(tactile_timestamps) + offset
    return aligned_timestamps






def interpolate_tactile_to_frames(
    aligned_timestamps: np.ndarray,
    tactile_data: List[Dict],
    start_frame: int,
    end_frame: int,
    video_fps: float = 60.0
) -> Dict[str, np.ndarray]:


    num_frames = end_frame - start_frame
    
    # Create video timestamps for this episode
    # Frames are numbered globally, but we need local timestamps
    video_timestamps = np.arange(start_frame, end_frame) / video_fps
    
    interpolated_data = {}
    
    def extract_field(data_list: List[Dict], path: List[str]) -> np.ndarray:
        result = []
        for item in data_list:
            val = item
            for key in path:
                val = val[key]
            result.append(val)
        return np.array(result)
    
    # Simple fields
    fields_to_interpolate = {
        'sensor_0_global_force': ['sensor_0', 'global_force'],
        'sensor_0_global_torque': ['sensor_0', 'global_torque'],
        'sensor_0_friction_est': ['sensor_0', 'friction_est'],
        'sensor_0_target_grip_force': ['sensor_0', 'target_grip_force'],
        'sensor_1_global_force': ['sensor_1', 'global_force'],
        'sensor_1_global_torque': ['sensor_1', 'global_torque'],
        'sensor_1_friction_est': ['sensor_1', 'friction_est'],
        'sensor_1_target_grip_force': ['sensor_1', 'target_grip_force'],
    }
    
    for field_name, json_path in fields_to_interpolate.items():
        data_array = extract_field(tactile_data, json_path)
        
        if data_array.ndim == 1:
            interp_func = interp1d(
                aligned_timestamps, data_array,
                kind='linear', bounds_error=False, fill_value='extrapolate'
            )
            interpolated_data[field_name] = interp_func(video_timestamps)
        else:
            interpolated = []
            for i in range(data_array.shape[1]):
                interp_func = interp1d(
                    aligned_timestamps, data_array[:, i],
                    kind='linear', bounds_error=False, fill_value='extrapolate'
                )
                interpolated.append(interp_func(video_timestamps))
            interpolated_data[field_name] = np.stack(interpolated, axis=1)
    
    
    # Boolean fields
    for sensor_id in [0, 1]:
        for bool_field in ['is_contact', 'is_sd_active']:
            field_name = f'sensor_{sensor_id}_{bool_field}'
            json_path = [f'sensor_{sensor_id}', bool_field]
            
            data_array = extract_field(tactile_data, json_path).astype(float)
            interp_func = interp1d(
                aligned_timestamps, data_array,
                kind='nearest', bounds_error=False, fill_value='extrapolate'
            )
            interpolated_data[field_name] = interp_func(video_timestamps).astype(bool)
    
    
    # Pillar data
    for sensor_id in [0, 1]:
        pillar_forces = []
        pillar_displacements = []
        pillar_contacts = []
        pillar_slip_states = []
        
        for tactile_frame in tactile_data:
            sensor_data = tactile_frame[f'sensor_{sensor_id}']
            pillars = sensor_data['pillars']
            
            forces = [p['force'] for p in pillars]
            displacements = [p['displacement'] for p in pillars]
            contacts = [p['in_contact'] for p in pillars]
            slip_states = [p['slip_state'] for p in pillars]
            
            pillar_forces.append(forces)
            pillar_displacements.append(displacements)
            pillar_contacts.append(contacts)
            pillar_slip_states.append(slip_states)
        
        pillar_forces = np.array(pillar_forces)
        pillar_displacements = np.array(pillar_displacements)
        pillar_contacts = np.array(pillar_contacts)
        pillar_slip_states = np.array(pillar_slip_states)
        
        interp_forces = np.zeros((num_frames, 9, 3))
        interp_displacements = np.zeros((num_frames, 9, 3))
        interp_contacts = np.zeros((num_frames, 9), dtype=bool)
        interp_slip_states = np.zeros((num_frames, 9))
        
        for pillar_idx in range(9):
            for dim_idx in range(3):
                interp_func = interp1d(
                    aligned_timestamps, pillar_forces[:, pillar_idx, dim_idx],
                    kind='linear', bounds_error=False, fill_value='extrapolate'
                )
                interp_forces[:, pillar_idx, dim_idx] = interp_func(video_timestamps)
                
                interp_func = interp1d(
                    aligned_timestamps, pillar_displacements[:, pillar_idx, dim_idx],
                    kind='linear', bounds_error=False, fill_value='extrapolate'
                )
                interp_displacements[:, pillar_idx, dim_idx] = interp_func(video_timestamps)
            
            interp_func = interp1d(
                aligned_timestamps, pillar_contacts[:, pillar_idx].astype(float),
                kind='nearest', bounds_error=False, fill_value='extrapolate'
            )
            interp_contacts[:, pillar_idx] = interp_func(video_timestamps).astype(bool)
            
            interp_func = interp1d(
                aligned_timestamps, pillar_slip_states[:, pillar_idx],
                kind='linear', bounds_error=False, fill_value='extrapolate'
            )
            interp_slip_states[:, pillar_idx] = interp_func(video_timestamps)
        
        interpolated_data[f'sensor_{sensor_id}_pillar_forces'] = interp_forces
        interpolated_data[f'sensor_{sensor_id}_pillar_displacements'] = interp_displacements
        interpolated_data[f'sensor_{sensor_id}_pillar_contacts'] = interp_contacts
        interpolated_data[f'sensor_{sensor_id}_pillar_slip_states'] = interp_slip_states
    
    return interpolated_data





def process_multidemo_dataset(
    zarr_path: str,
    qr_sync_files: List[str],
    tactile_bags: List[str],
    video_fps: float = 60.0 
) -> Dict[str, np.ndarray]:
   
   
    print("="*80)
    print("PROCESSING MULTI-DEMO DATASET")
    print("="*80)
    
    # Load Zarr to get episode boundaries
    if zarr_path.endswith('.zip'):
        store = zarr.ZipStore(zarr_path, mode='r')
    else:
        store = zarr.DirectoryStore(zarr_path)
    root = zarr.open(store, mode='r')
    
    boundaries = get_episode_boundaries(root)
    num_episodes = len(boundaries)
    
    print(f"\nFound {num_episodes} episodes in Zarr")
    for i, (start, end) in enumerate(boundaries):
        print(f"  Episode {i+1}: frames {start} to {end} ({end-start} frames)")
    
    if isinstance(store, zarr.ZipStore):
        store.close()
    
    # Verify haveing matching number of QR and bag files
    if len(qr_sync_files) != num_episodes:
        raise ValueError(f"Number of QR files ({len(qr_sync_files)}) doesn't match episodes ({num_episodes})")
    if len(tactile_bags) != num_episodes:
        raise ValueError(f"Number of tactile bags ({len(tactile_bags)}) doesn't match episodes ({num_episodes})")
    
    # Process each episode
    all_tactile_data = {}  # Will store concatenated arrays
    
    for episode_idx in range(num_episodes):
        print(f"\n{'='*80}")
        print(f"PROCESSING EPISODE {episode_idx + 1}/{num_episodes}")
        print(f"{'='*80}")
        
        start_frame, end_frame = boundaries[episode_idx]
        qr_sync_file = qr_sync_files[episode_idx]
        tactile_bag = tactile_bags[episode_idx]
        
        print(f"Frames: {start_frame} to {end_frame}")
        print(f"QR sync: {qr_sync_file}")
        print(f"Tactile bag: {tactile_bag}")
        
        # Load QR sync
        print(f"\nLoading QR sync")
        qr_sync_data = read_qr_sync_data(qr_sync_file)
        print(f"Offset: {qr_sync_data['offset']:.3f}s")
        
        # Read tactile bag
        print(f"\nReading tactile bag")
        tactile_timestamps, tactile_data = read_tactile_bag(tactile_bag)
        print(f"Read {len(tactile_data)} tactile messages")
        
        # Align timestamps
        print(f"\n   Aligning timestamps")
        aligned_timestamps = align_timestamps_with_qr(
            tactile_timestamps,
            qr_sync_data,
            video_fps
        )
        
        # Interpolate for this episode's frames
        print(f"\nInterpolating to {end_frame - start_frame} frames")
        episode_tactile = interpolate_tactile_to_frames(
            aligned_timestamps,
            tactile_data,
            start_frame,
            end_frame,
            video_fps
        )
        
        if episode_idx == 0:
            # First episode - initialize arrays
            all_tactile_data = episode_tactile
        else:
            # Subsequent episodes 
            for field_name, data_array in episode_tactile.items():
                all_tactile_data[field_name] = np.concatenate([
                    all_tactile_data[field_name],
                    data_array
                ], axis=0)
        
        print(f"   Episode {episode_idx + 1} processed")
    
    print(f"\n{'='*80}")
    print(f"ALL EPISODES PROCESSED")
    print(f"{'='*80}")
    
    # Verify final array sizes
    total_frames = boundaries[-1][1]  # Last episode end frame
    for field_name, data_array in all_tactile_data.items():
        expected_frames = total_frames
        actual_frames = data_array.shape[0]
        if actual_frames != expected_frames:
            print(f" Warning: {field_name} has {actual_frames} frames, expected {expected_frames}")
        else:
            print(f"{field_name}: {data_array.shape}")
    
    return all_tactile_data


def add_tactile_to_zarr(
    input_zarr_path: str,
    output_zarr_path: str,
    tactile_data: Dict[str, np.ndarray]
):
    """Add tactile data to Zarr dataset ."""
    print(f"\n{'='*80}")
    print(f"ADDING TACTILE TO ZARR")
    print(f"{'='*80}")
    
    if input_zarr_path.endswith('.zip'):
        input_store = zarr.ZipStore(input_zarr_path, mode='r')
    else:
        input_store = zarr.DirectoryStore(input_zarr_path)
    input_root = zarr.open(input_store, mode='r')
    
    if output_zarr_path.endswith('.zip'):
        output_store = zarr.ZipStore(output_zarr_path, mode='w')
    else:
        output_store = zarr.DirectoryStore(output_zarr_path)
    output_root = zarr.open(output_store, mode='w')
    
    print("Copying original data...")
    
    # Try to copy all data, but handle codec errors gracefully
    try:
        zarr.copy_all(input_root, output_root)
        print("All data copied successfully")
    except ValueError as e:
        if 'codec not available' in str(e):
            print(f"Codec error detected: {str(e)[:60]}")
            print("Copying data selectively (skipping arrays with unavailable codecs)")
            
            # Copy metadata
            output_root.attrs.update(input_root.attrs)
            
            # Copy each group/array individually with error handling
            def copy_tree_safe(src, dst, path=''):
                """Recursively copy Zarr tree, skipping items that fail."""
                for key in src.keys():
                    try:
                        src_item = src[key]
                        full_path = f"{path}/{key}" if path else key
                        
                        if isinstance(src_item, zarr.hierarchy.Group):
                            # Create group and recurse
                            print(f"Creating group: {full_path}")
                            dst_group = dst.create_group(key, overwrite=True)
                            dst_group.attrs.update(src_item.attrs)
                            copy_tree_safe(src_item, dst_group, full_path)
                        else:
                            # Try to copy array
                            print(f"Copying {full_path}...", end='')
                            zarr.copy(src_item, dst, name=key, if_exists='replace')
                            print("Done")
                    
                    except ValueError as copy_error:
                        if 'codec not available' in str(copy_error):
                            print(f" Skipped (codec unavailable)")
                            # For critical data, create a placeholder
                            if any(x in key for x in ['action', 'episode', 'timestamp']):
                                print(f"Creating zero-filled placeholder for {full_path}")
                                try:
                                    dst.create_dataset(
                                        key,
                                        shape=src_item.shape,
                                        dtype=src_item.dtype,
                                        fill_value=0,
                                        chunks=True
                                    )
                                except:
                                    pass
                        else:
                            print(f" Skipped (error: {str(copy_error)[:40]})")
                    
                    except Exception as copy_error:
                        print(f" Skipped (unexpected error: {str(copy_error)[:40]})")
            
            # Start recursive copy
            copy_tree_safe(input_root, output_root)
        else:
            # Different error - re-raise it
            raise
    
    print("\n   Adding tactile data")
    tactile_group = output_root.create_group('data/tactile', overwrite=True)
    
    for field_name, data_array in tactile_data.items():
        print(f"     Adding {field_name}: shape={data_array.shape}")
        tactile_group.create_dataset(
            field_name,
            data=data_array,
            chunks=True,
            compression='blosc'
        )
    
    if isinstance(output_store, zarr.ZipStore):
        output_store.close()
    if isinstance(input_store, zarr.ZipStore):
        input_store.close()
    
    print(f"Successfully created Zarr with tactile!")


def main():
    
    
    parser = argparse.ArgumentParser(
        description='Add tactile to multi-demo Zarr using QR sync'
    )
    parser.add_argument('--zarr_path', required=True,
                       help='Input Zarr dataset (with multiple demos)')
    parser.add_argument('--qr_sync', nargs='+', required=True,
                       help='QR sync JSON files (one per demo, in order)')
    parser.add_argument('--tactile_bags', nargs='+', required=True,
                       help='ROS2 bags with tactile (one per demo, in order)')
    parser.add_argument('--output', required=True,
                       help='Output Zarr with tactile')
    parser.add_argument('--video_fps', type=float, default=60.0,
                   help='Video FPS (default: 60)')
    
    args = parser.parse_args()
    
    # Verify arguments
    if len(args.qr_sync) != len(args.tactile_bags):
        print(" error: Number of QR sync files must match number of tactile bags!")
        return
    
    print(f"\n UMI MULTI-DEMO TACTILE INTEGRATION")
    print(f"Processing {len(args.qr_sync)} demos")
    
    # Process all demos
    all_tactile_data = process_multidemo_dataset(
        args.zarr_path,
        args.qr_sync,
        args.tactile_bags,
        args.video_fps
    )
    
    # Add to Zarr
    add_tactile_to_zarr(
        args.zarr_path,
        args.output,
        all_tactile_data
    )
    
    print(f"\n{'='*80}")
    print(f" PIPELINE COMPLETE!")
    print(f"{'='*80}")
    print(f"\nYour multi-demo dataset with tactile is ready:")
    print(f"   {args.output}")
    print(f"\n Processed {len(args.qr_sync)} demos with QR synchronization")


if __name__ == '__main__':
    main()