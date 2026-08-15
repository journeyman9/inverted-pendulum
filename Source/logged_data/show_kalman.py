import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

if __name__ == "__main__":
    df_kalman = pd.read_csv("./normal_operation/result-data-timestamps-200-001.csv")

    # Create 7 subplots sharing the x-axis (timestamp or iterations)
    fig, axs = plt.subplots(7, 1, figsize=(10, 18), sharex=True)

    # Use timestamp or index as x-axis; here using 'timestamp' if available, or row index
    x_axis = df_kalman['timestamp'] if 'timestamp' in df_kalman.columns else df_kalman.index

    # 1. State 0 error: x_hat_0 - x
    axs[0].plot(x_axis, df_kalman['x_hat_0'] - df_kalman['x'], label=r'$\hat{x} - x$', color='tab:blue')
    axs[0].set_ylabel(r'$\hat{x} - x$')
    axs[0].legend(loc='upper right')

    # 2. State 1 error: x_hat_1 - xdot
    axs[1].plot(x_axis, df_kalman['x_hat_1'] - df_kalman['xdot'], label=r'$\hat{\dot{x}} - \dot{x}$', color='tab:orange')
    axs[1].set_ylabel(r'$\hat{\dot{x}} - \dot{x}$')
    axs[1].legend(loc='upper right')

    # 3. State 2 error: x_hat_2 - theta
    axs[2].plot(x_axis, df_kalman['x_hat_2'] - df_kalman['theta'], label=r'$\hat{\theta} - \theta$', color='tab:green')
    axs[2].set_ylabel(r'$\hat{\theta} - \theta$')
    axs[2].legend(loc='upper right')

    # 4. State 3 error: x_hat_3 - thetadot
    axs[3].plot(x_axis, df_kalman['x_hat_3'] - df_kalman['thetadot'], label=r'$\hat{\dot{\theta}} - \dot{\theta}$', color='tab:red')
    axs[3].set_ylabel(r'$\hat{\dot{\theta}} - \dot{\theta}$')
    axs[3].legend(loc='upper right')

    # 5. Control input u
    axs[4].plot(x_axis, df_kalman['u'], label=r'$u$', color='tab:purple')
    axs[4].set_ylabel(r'Control $u$')
    axs[4].legend(loc='upper right')

    # 6. Trace of Kalman Gain filter (trace_Kf)
    axs[5].plot(x_axis, df_kalman['trace_Kf'], label=r'Trace $K_f$', color='tab:brown')
    axs[5].set_ylabel(r'Trace $K_f$')
    axs[5].legend(loc='upper right')

    # 7. Trace of Error Covariance (trace_P)
    axs[6].plot(x_axis, df_kalman['trace_P'], label=r'Trace $P$', color='tab:pink')
    axs[6].set_ylabel(r'Trace $P$')
    axs[6].set_xlabel('Timestamp / Iterations')
    axs[6].legend(loc='upper right')

    plt.tight_layout()

    # Create 7 subplots sharing the x-axis (timestamp or iterations)
    fig, axs = plt.subplots(7, 1, figsize=(10, 18), sharex=True)

    # Use timestamp or index as x-axis; here using 'timestamp' if available, or row index
    x_axis = df_kalman['timestamp'] if 'timestamp' in df_kalman.columns else df_kalman.index

    # 1. State 0: x_hat_0 vs x
    axs[0].plot(x_axis, df_kalman['x_hat_0'], label=r'$\hat{x}$ (Estimate)', color='tab:blue')
    axs[0].plot(x_axis, df_kalman['x'], label=r'$x$ (Raw)', color='tab:cyan', linestyle='--')
    axs[0].set_ylabel(r'State $x$')
    axs[0].legend(loc='upper right')

    # 2. State 1: x_hat_1 vs xdot
    axs[1].plot(x_axis, df_kalman['x_hat_1'], label=r'$\hat{\dot{x}}$ (Estimate)', color='tab:orange')
    axs[1].plot(x_axis, df_kalman['xdot'], label=r'$\dot{x}$ (Raw)', color='gold', linestyle='--')
    axs[1].set_ylabel(r'State $\dot{x}$')
    axs[1].legend(loc='upper right')

    # 3. State 2: x_hat_2 vs theta
    axs[2].plot(x_axis, df_kalman['x_hat_2'], label=r'$\hat{\theta}$ (Estimate)', color='tab:green')
    axs[2].plot(x_axis, df_kalman['theta'], label=r'$\theta$ (Raw)', color='lightgreen', linestyle='--')
    axs[2].set_ylabel(r'State $\theta$')
    axs[2].legend(loc='upper right')

    # 4. State 3: x_hat_3 vs thetadot
    axs[3].plot(x_axis, df_kalman['x_hat_3'], label=r'$\hat{\dot{\theta}}$ (Estimate)', color='tab:red')
    axs[3].plot(x_axis, df_kalman['thetadot'], label=r'$\dot{\theta}$ (Raw)', color='salmon', linestyle='--')
    axs[3].set_ylabel(r'State $\dot{\theta}$')
    axs[3].legend(loc='upper right')

    # 5. Control input u
    axs[4].plot(x_axis, df_kalman['u'], label=r'$u$', color='tab:purple')
    axs[4].set_ylabel(r'Control $u$')
    axs[4].legend(loc='upper right')

    # 6. Trace of Kalman Gain filter (trace_Kf)
    axs[5].plot(x_axis, df_kalman['trace_Kf'], label=r'Trace $K_f$', color='tab:brown')
    axs[5].set_ylabel(r'Trace $K_f$')
    axs[5].legend(loc='upper right')

    # 7. Trace of Error Covariance (trace_P)
    axs[6].plot(x_axis, df_kalman['trace_P'], label=r'Trace $P$', color='tab:pink')
    axs[6].set_ylabel(r'Trace $P$')
    axs[6].set_xlabel('Timestamp / Iterations')
    axs[6].legend(loc='upper right')

    plt.tight_layout()
    plt.show()
