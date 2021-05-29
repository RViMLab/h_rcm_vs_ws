
import os
import sys
import pandas as pd
import cv2

sys.path.append('/opt/ros/melodic/lib/python2.7/dist-packages')
sys.path.append('/home/martin/Control/h_rcom_vs_ws/devel/lib/python2.7/dist-packages')


def h_rcm_vs_csv_exporter(df: pd.DataFrame, log_dir: str, out_file: str):
    export_df = pd.DataFrame()
    export_df['time'] = df.time
    # states
    export_df['states.task.values'] = df.rcmFeedback.apply(lambda x: x.states.task.values)
    export_df['states.p_trocar.position.x'] = df.rcmFeedback.apply(lambda x: x.states.p_trocar.position.x)
    export_df['states.p_trocar.position.y'] = df.rcmFeedback.apply(lambda x: x.states.p_trocar.position.y)
    export_df['states.p_trocar.position.z'] = df.rcmFeedback.apply(lambda x: x.states.p_trocar.position.z)
    # errors
    export_df['errors.task.values'] = df.rcmFeedback.apply(lambda x: x.errors.task.values)
    export_df['errors.p_trocar.position.x'] = df.rcmFeedback.apply(lambda x: x.errors.p_trocar.position.x)
    export_df['errors.p_trocar.position.y'] = df.rcmFeedback.apply(lambda x: x.errors.p_trocar.position.y)
    export_df['errors.p_trocar.position.z'] = df.rcmFeedback.apply(lambda x: x.errors.p_trocar.position.z)

    output = os.path.join(log_dir, out_file)
    export_df.to_csv(output, index=False)

def pairwise_distance_csv_exporter(df: pd.DataFrame, log_dir: str, out_file: str):
    export_df = pd.DataFrame()
    export_df['time'] = df.time
    export_df['mean_pairwise_distance'] = df.pairwise_distance.apply(lambda x: x.mean_pairwise_distance.data)
    export_df['std_pairwise_distance'] = df.pairwise_distance.apply(lambda x: x.std_pairwise_distance.data)
    export_df['n_matches'] = df.pairwise_distance.apply(lambda x: x.n_matches.data)
    
    output = os.path.join(log_dir, out_file)
    export_df.to_csv(output, index=False)

def path_csv_exporter(df: pd.DataFrame, log_dir: str, out_file: str):
    export_df = pd.DataFrame()
    export_df['time'] = df.time
    export_df['id'] = df.id
    export_df['path'] = df.path
    export_df['target_rcm.task.values'] = df.target_rcm.apply(lambda x: x.task.values)
    export_df['target_rcm.p_trocar_position.x'] = df.target_rcm.apply(lambda x: x.p_trocar.position.x)
    export_df['target_rcm.p_trocar_position.y'] = df.target_rcm.apply(lambda x: x.p_trocar.position.y)
    export_df['target_rcm.p_trocar_position.z'] = df.target_rcm.apply(lambda x: x.p_trocar.position.z)
    export_df['final_rcm.task.values'] = df.final_rcm.apply(lambda x: x.task.values)
    export_df['final_rcm.p_trocar_position.x'] = df.final_rcm.apply(lambda x: x.p_trocar.position.x)
    export_df['final_rcm.p_trocar_position.y'] = df.final_rcm.apply(lambda x: x.p_trocar.position.y)
    export_df['final_rcm.p_trocar_position.z'] = df.final_rcm.apply(lambda x: x.p_trocar.position.z)
    export_df['target_joint_state.position'] = df.target_joint_state.apply(lambda x: x.position)
    export_df['final_joint_state.position'] = df.final_joint_state.apply(lambda x: x.position)

    # export images
    path = out_file.split('/')[-1].split('.')[0]
    for _, row in df.iterrows():
        if not os.path.exists(os.path.join(log_dir, path)):
            print('Creating directory {}'.format(os.path.join(log_dir, path)))
            os.mkdir(os.path.join(log_dir, path))
        cv2.imwrite(
            os.path.join(log_dir, path, 'target_img_id_{}.png'.format(row.id)), 
            row.target_img
        )
        cv2.imwrite(
            os.path.join(log_dir, path, 'final_img_id_{}.png'.format(row.id)), 
            row.final_img
        )

    output = os.path.join(log_dir, out_file)
    export_df.to_csv(output)

def rcm_state_csv_exporter(df: pd.DataFrame, log_dir: str, out_file: str):
    export_df = pd.DataFrame()
    export_df['time'] = df.time
    export_df['task.values'] = df.rcm.apply(lambda x: x.task.values)
    export_df['p_trocar.position.x'] = df.rcm.apply(lambda x: x.p_trocar.position.x)
    export_df['p_trocar.position.y'] = df.rcm.apply(lambda x: x.p_trocar.position.y)
    export_df['p_trocar.position.z'] = df.rcm.apply(lambda x: x.p_trocar.position.z)

    output = os.path.join(log_dir, out_file)
    export_df.to_csv(output, index=False)

def twist_csv_exporter(df: pd.DataFrame, log_dir: str, out_file: str):
    export_df = pd.DataFrame()
    export_df['time'] = df.time
    export_df['linear.x'] = df.Twist.apply(lambda x: x.linear.x)
    export_df['linear.y'] = df.Twist.apply(lambda x: x.linear.y)
    export_df['linear.z'] = df.Twist.apply(lambda x: x.linear.z)
    export_df['angular.x'] = df.Twist.apply(lambda x: x.angular.x)
    export_df['angular.y'] = df.Twist.apply(lambda x: x.angular.y)
    export_df['angular.z'] = df.Twist.apply(lambda x: x.angular.z)
    
    output = os.path.join(log_dir, out_file)
    export_df.to_csv(output, index=False)

def joint_state_csv_exporter(df: pd.DataFrame, log_dir: str, out_file: str):
    exporter_df = pd.DataFrame()
    exporter_df['time'] = df.time
    exporter_df['JointState.position'] = df.JointState.apply(lambda x: x.position)
    exporter_df['JointState.name'] = df.JointState.apply(lambda x: x.name)
    
    output = os.path.join(log_dir, out_file)
    exporter_df.to_csv(output, index=False)

def exporter(prefix: str, files: dict, log_dir: str):
    for key, value in files.items():
        out_file = '{}.csv'.format(key.split('.')[0])
        print('Exporter writing file {} to {}'.format(out_file, log_dir))
        df = pd.read_pickle(os.path.join(prefix, key))

        if value is 'h_rcm_vs':
            h_rcm_vs_csv_exporter(df, log_dir, out_file)
        elif value is 'pairwise_distance':
            pairwise_distance_csv_exporter(df, log_dir, out_file)
        elif value is 'path':
            path_csv_exporter(df, log_dir, out_file)
        elif value is 'rcm_state':
            rcm_state_csv_exporter(df, log_dir, out_file)
        elif value is 'twist':
            twist_csv_exporter(df, log_dir, out_file)
        elif value is 'joint_state':
            joint_state_csv_exporter(df, log_dir, out_file)
        else:
            print('Unsuported type found.')


if __name__ == '__main__':

    prefix = '/home/martin/Measurements/21_05_29_tool_insertion_to_close_up/05'
    files = {
        'h_rcm_vs_feedback.pkl': 'h_rcm_vs',
        'pairwise_distance.pkl': 'pairwise_distance',
        'path_0.pkl': 'path',
        'path_1.pkl': 'path',
        'rcm_state.pkl': 'rcm_state',
        'twist.pkl': 'twist',
        'joint_state.pkl': 'joint_state'
    }
    log_dir = '/home/martin/Measurements/21_05_29_tool_insertion_to_close_up/05/exports'
    exporter(prefix, files, log_dir)
