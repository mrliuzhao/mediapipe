package com.google.mediapipe.apps.poc;

import android.content.pm.ApplicationInfo;
import android.content.pm.PackageManager;
import android.content.pm.PackageManager.NameNotFoundException;
import android.graphics.SurfaceTexture;
import android.media.MediaPlayer;
import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;
import android.os.SystemClock;
import android.util.Log;
import android.util.Size;
import android.view.SurfaceHolder;
import android.view.SurfaceView;
import android.view.View;
import android.widget.Chronometer;
import android.widget.TextView;

import com.google.mediapipe.apps.poc.R;
import com.google.mediapipe.components.CameraHelper;
import com.google.mediapipe.components.CameraXPreviewHelper;
import com.google.mediapipe.components.ExternalTextureConverter;
import com.google.mediapipe.components.FrameProcessor;
import com.google.mediapipe.components.PermissionHelper;
import com.google.mediapipe.framework.AndroidAssetUtil;
import com.google.mediapipe.framework.PacketGetter;
import com.google.mediapipe.glutil.EglManager;
import com.google.protobuf.InvalidProtocolBufferException;

import com.google.mediapipe.formats.proto.LandmarkProto.NormalizedLandmark;
import com.google.mediapipe.formats.proto.LandmarkProto.NormalizedLandmarkList;

import java.util.List;
import java.util.Locale;

import androidx.appcompat.app.AppCompatActivity;

/** Main activity of MediaPipe basic app. */
public class MainActivity extends AppCompatActivity {
  private static final String TAG = "MainActivity";

  /**
   * Flips the camera-preview frames vertically by default, before sending them into FrameProcessor
   * to be processed in a MediaPipe graph, and flips the processed frames back when they are
   * displayed. This maybe needed because OpenGL represents images assuming the image origin is at
   * the bottom-left corner, whereas MediaPipe in general assumes the image origin is at the
   * top-left corner.
   * NOTE: use "flipFramesVertically" in manifest metadata to override this behavior.
   */
  private static final boolean FLIP_FRAMES_VERTICALLY = true;

  /**
   * Number of output frames allocated in ExternalTextureConverter.
   * NOTE: use "converterNumBuffers" in manifest metadata to override number of buffers. For
   * example, when there is a FlowLimiterCalculator in the graph, number of buffers should be at
   * least `max_in_flight + max_in_queue + 1` (where max_in_flight and max_in_queue are used in
   * FlowLimiterCalculator options). That's because we need buffers for all the frames that are in
   *  flight/queue plus one for the next frame from the camera.
   */
  private static final int NUM_BUFFERS = 2;

  private static final String OUTPUT_LANDMARKS_STREAM_NAME = "pose_landmarks_every";

  private static final int CAMERA_WIDTH = 640;
  private static final int CAMERA_HEIGHT = 480;

  // Load all native libraries needed by the app.
  static {
    System.loadLibrary("mediapipe_jni");
    try {
      System.loadLibrary("opencv_java3");
    } catch (UnsatisfiedLinkError e) {
      // Some example apps (e.g. template matching) require OpenCV 4.
      System.loadLibrary("opencv_java4");
    }
  }

  /**
   * Main UI handler
   */
  private Handler mainHandler = new Handler(Looper.getMainLooper());

  /**
   * Sends camera-preview frames into a MediaPipe graph for processing, and displays the processed
   * frames onto a {@link android.view.Surface}
   */
  protected FrameProcessor processor;

  /**
   * Handles camera access via the {@link androidx.camera.core.CameraX} Jetpack support library.
   */
  protected CameraXPreviewHelper cameraHelper;

  /**
   * {@link SurfaceTexture} where the camera-preview frames can be accessed
   */
  private SurfaceTexture previewFrameTexture;

  /**
   * {@link SurfaceView} that displays the camera-preview frames processed by a MediaPipe graph.
   */
  private SurfaceView cameraDisplaySurfaceView;

  /**
   * {@link SurfaceView} that displays the training video.
   */
  private SurfaceView trainDisplaySurfaceView;
  /**
   * Player for training video
   */
  private MediaPlayer trainPlayer;

  /**
   * 左右手外展角度的文本
   */
  private TextView leftAbdText, rightAbdText;
  /**
   * 训练计时器
   */
  private Chronometer trainTimer;

  /**
   * Creates and manages an {@link android.opengl.EGLContext}
   */
  private EglManager eglManager;

  /**
   * Converts the GL_TEXTURE_EXTERNAL_OES texture from Android camera into a regular texture to be
   * consumed by {@link FrameProcessor} and the underlying MediaPipe graph.
   */
  private ExternalTextureConverter converter;

  /**
   * ApplicationInfo for retrieving metadata defined in the manifest.
   */
  private ApplicationInfo applicationInfo;

  @Override
  protected void onCreate(Bundle savedInstanceState) {
    super.onCreate(savedInstanceState);
    setContentView(R.layout.activity_poc_demo);

    try {
      applicationInfo =
              getPackageManager().getApplicationInfo(getPackageName(), PackageManager.GET_META_DATA);
    } catch (NameNotFoundException e) {
      Log.e(TAG, "Cannot find application info: " + e);
    }

    // 右侧进行摄像头展示
    cameraDisplaySurfaceView = findViewById(R.id.content_right);
    setupCameraDisplay();

    // Initialize asset manager so that MediaPipe native libraries can access the app assets, e.g.,
    // binary graphs.
    AndroidAssetUtil.initializeNativeAssetManager(this);
    eglManager = new EglManager(null);
    processor = new FrameProcessor(this, eglManager.getNativeContext(),
                    applicationInfo.metaData.getString("binaryGraphName"),
                    applicationInfo.metaData.getString("inputVideoStreamName"),
                    applicationInfo.metaData.getString("outputVideoStreamName"));
    processor.getVideoSurfaceOutput()
            .setFlipY(applicationInfo.metaData.getBoolean("flipFramesVertically", FLIP_FRAMES_VERTICALLY));

    PermissionHelper.checkAndRequestCameraPermissions(this);

    leftAbdText = findViewById(R.id.left_abduction);
    rightAbdText = findViewById(R.id.right_abduction);
    processor.addPacketCallback(
            OUTPUT_LANDMARKS_STREAM_NAME,
            (packet) -> {
              try {
                // Parse from raw data
                byte[] landmarksRaw = PacketGetter.getProtoBytes(packet);
                NormalizedLandmarkList poseLandmarks = NormalizedLandmarkList.parseFrom(landmarksRaw);
                // 没有关节点时重置角度文字
                if (poseLandmarks.getLandmarkCount() == 0) {
                  mainHandler.post(() -> {
                    leftAbdText.setText("左臂外展：N/A");
                    rightAbdText.setText("右臂外展：N/A");
                  });
                } else {
                  List<NormalizedLandmark> lmList = poseLandmarks.getLandmarkList();
                  NormalizedLandmark left_shoulder = lmList.get(11);
                  NormalizedLandmark right_shoulder = lmList.get(12);
                  NormalizedLandmark left_elbow = lmList.get(13);
                  NormalizedLandmark right_elbow = lmList.get(14);

                  // 仅对可见的手臂进行计算
                  if ((left_shoulder.hasVisibility() && left_shoulder.getVisibility() < 0.6)
                          || (left_elbow.hasVisibility() && left_elbow.getVisibility() < 0.6)) {
                    mainHandler.post(() -> {
                      leftAbdText.setText("左臂外展：N/A");
                    });
                  } else {
                    double ls_x = left_shoulder.getX() * CAMERA_WIDTH;
                    double ls_y = left_shoulder.getY() * CAMERA_HEIGHT;
                    double le_x = left_elbow.getX() * CAMERA_WIDTH;
                    double le_y = left_elbow.getY() * CAMERA_HEIGHT;
                    double cos_left = (le_y - ls_y) /
                            Math.sqrt(Math.pow((le_x - ls_x), 2.0) + Math.pow((le_y - ls_y), 2.0));
                    // 防止underflow
                    if (Math.abs(cos_left - 1.0) < 0.0001){
                      cos_left = 1.0;
                    }
                    if (Math.abs(cos_left + 1.0) < 0.0001) {
                      cos_left = -1.0;
                    }

                    double left_abd = Math.acos(cos_left) * 180.0 / Math.PI;
                    mainHandler.post(() -> {
                      leftAbdText.setText(String.format(Locale.CHINESE, "左臂外展：%.1f°", left_abd));
                    });
                  }

                  if ((right_shoulder.hasVisibility() && right_shoulder.getVisibility() < 0.6)
                          || (right_elbow.hasVisibility() && right_elbow.getVisibility() < 0.6)) {
                    mainHandler.post(() -> {
                      rightAbdText.setText("右臂外展：N/A");
                    });
                  } else {
                    double rs_x = right_shoulder.getX() * CAMERA_WIDTH;
                    double rs_y = right_shoulder.getY() * CAMERA_HEIGHT;
                    double re_x = right_elbow.getX() * CAMERA_WIDTH;
                    double re_y = right_elbow.getY() * CAMERA_HEIGHT;

                    double cos_right = (re_y - rs_y) /
                            Math.sqrt(Math.pow((re_x - rs_x), 2.0) + Math.pow((re_y - rs_y), 2.0));
                    // 防止underflow
                    if (Math.abs(cos_right - 1.0) < 0.0001) {
                      cos_right = 1.0;
                    }
                    if (Math.abs(cos_right + 1.0) < 0.0001) {
                      cos_right = -1.0;
                    }

                    double right_abd = Math.acos(cos_right) * 180.0 / Math.PI;
                    mainHandler.post(() -> {
                      rightAbdText.setText(String.format(Locale.CHINESE, "右臂外展：%.1f°", right_abd));
                    });
                  }
                }

              } catch (InvalidProtocolBufferException exception) {
                Log.e(TAG, "Failed to get proto.", exception);
              }
            });

    // 左侧播放视频
    trainDisplaySurfaceView = findViewById(R.id.content_left);
    setupTrainDisplay();

    // 设置训练计时器
    trainTimer = findViewById(R.id.train_time);
    trainTimer.setBase(SystemClock.elapsedRealtime());
    trainTimer.start();

  }

  private void setupTrainDisplay() {
//    trainDisplaySurfaceView.setVisibility(View.INVISIBLE);

    trainDisplaySurfaceView.getHolder()
            .addCallback(new SurfaceHolder.Callback() {
              @Override
              public void surfaceCreated(SurfaceHolder holder) {
                trainPlayer = MediaPlayer.create(MainActivity.this, R.raw.anim2);
                if (trainPlayer == null) {
                  Log.e(TAG, "trainPlayer fail to open!!!!");
                }
                trainPlayer.setLooping(true);  // 设置循环播放
                trainPlayer.setDisplay(holder);

                trainPlayer.start(); // 开始播放
                Log.e(TAG, "start to play train video!!!!");

//                trainDisplaySurfaceView.setVisibility(View.VISIBLE);
              }

              @Override
              public void surfaceChanged(SurfaceHolder holder, int format, int width, int height) {
              }

              @Override
              public void surfaceDestroyed(SurfaceHolder holder) {
              }
            });
  }

  @Override
  protected void onResume() {
    super.onResume();
    converter =
            new ExternalTextureConverter(
                    eglManager.getContext(),
                    applicationInfo.metaData.getInt("converterNumBuffers", NUM_BUFFERS));
    converter.setFlipY(
            applicationInfo.metaData.getBoolean("flipFramesVertically", FLIP_FRAMES_VERTICALLY));
    converter.setConsumer(processor);
    if (PermissionHelper.cameraPermissionsGranted(this)) {
      startCamera();
    }
  }

  @Override
  protected void onPause() {
    super.onPause();
    converter.close();

    // Hide preview display until we re-open the camera again.
    cameraDisplaySurfaceView.setVisibility(View.INVISIBLE);
  }

  @Override
  public void onRequestPermissionsResult(
          int requestCode, String[] permissions, int[] grantResults) {
    super.onRequestPermissionsResult(requestCode, permissions, grantResults);
    PermissionHelper.onRequestPermissionsResult(requestCode, permissions, grantResults);
  }

  protected void onCameraStarted(SurfaceTexture surfaceTexture) {
    previewFrameTexture = surfaceTexture;
    // Make the display view visible to start showing the preview. This triggers the
    // SurfaceHolder.Callback added to (the holder of) previewDisplayView.
    cameraDisplaySurfaceView.setVisibility(View.VISIBLE);
  }

  protected Size cameraTargetResolution() {
    return new Size(CAMERA_WIDTH, CAMERA_HEIGHT);
  }

  public void startCamera() {
    cameraHelper = new CameraXPreviewHelper();
    cameraHelper.setOnCameraStartedListener(
            surfaceTexture -> {
              onCameraStarted(surfaceTexture);
            });
    CameraHelper.CameraFacing cameraFacing =
            applicationInfo.metaData.getBoolean("cameraFacingFront", false)
                    ? CameraHelper.CameraFacing.FRONT
                    : CameraHelper.CameraFacing.BACK;
    cameraHelper.startCamera(
            this, cameraFacing, /*unusedSurfaceTexture=*/ null, cameraTargetResolution());
  }

  protected Size computeViewSize(int width, int height) {
    return new Size(width, height);
  }

  protected void onPreviewDisplaySurfaceChanged(
          SurfaceHolder holder, int format, int width, int height) {
    // (Re-)Compute the ideal size of the camera-preview display (the area that the
    // camera-preview frames get rendered onto, potentially with scaling and rotation)
    // based on the size of the SurfaceView that contains the display.
    Size viewSize = computeViewSize(width, height);
    Size displaySize = cameraHelper.computeDisplaySizeFromViewSize(viewSize);
    boolean isCameraRotated = cameraHelper.isCameraRotated();

    // Connect the converter to the camera-preview frames as its input (via
    // previewFrameTexture), and configure the output width and height as the computed
    // display size.
    converter.setSurfaceTextureAndAttachToGLContext(
            previewFrameTexture,
            isCameraRotated ? displaySize.getHeight() : displaySize.getWidth(),
            isCameraRotated ? displaySize.getWidth() : displaySize.getHeight());
  }

  private void setupCameraDisplay() {
    cameraDisplaySurfaceView.setVisibility(View.INVISIBLE);
    cameraDisplaySurfaceView
            .getHolder()
            .addCallback(
                    new SurfaceHolder.Callback() {
                      @Override
                      public void surfaceCreated(SurfaceHolder holder) {
                        processor.getVideoSurfaceOutput().setSurface(holder.getSurface());
                      }

                      @Override
                      public void surfaceChanged(SurfaceHolder holder, int format, int width, int height) {
                        onPreviewDisplaySurfaceChanged(holder, format, width, height);
                      }

                      @Override
                      public void surfaceDestroyed(SurfaceHolder holder) {
                        processor.getVideoSurfaceOutput().setSurface(null);
                      }
                    });
  }

  @Override
  protected void onDestroy() {
    super.onDestroy();

    // 关闭训练视频
    if (trainPlayer != null) {
      if (trainPlayer.isPlaying()) {
        trainPlayer.stop();
      }
      trainPlayer.release();
      Log.e(TAG, "Successfully release train player!!!!");
    } else {
      Log.d(TAG, "trainPlayer is null, no need to destroy!!!!");
    }

  }
}
