﻿using Microsoft.Kinect;
using Kinect.Toolbox;
using System.IO;
using System.Speech.Synthesis;
using System.Speech.Recognition;
using System.Windows;
using System.Windows.Media;
using System.Windows.Media.Imaging;

namespace Kinect
{

    public partial class MainWindow : Window {

        private System.Speech.Recognition.SpeechRecognitionEngine engine = new SpeechRecognitionEngine();
        private SpeechSynthesizer synthesizer = new SpeechSynthesizer();
        private KinectSensor sensor;
        private WriteableBitmap bitmap;
        private DepthImagePixel[] depthPixels;
        private DrawingGroup drawingGroup;
        private DrawingImage imageSource;
        private Skeleton[] skeletons;
        private byte[] pixels;
        private const float RenderWidth = 640.0f;
        private const float RenderHeight = 480.0f;
        private const double JointThickness = 3;
        private const double BodyCenterThickness = 10;
        private const double ClipBoundsThickness = 10;
        private readonly Brush centerPointBrush = Brushes.Blue;
        private readonly Brush trackedJointBrush = new SolidColorBrush(Color.FromArgb(255, 68, 192, 68));
        private readonly Brush inferredJointBrush = Brushes.Yellow;
        private readonly Pen trackedBonePen = new Pen(Brushes.Green, 6);
        private readonly Pen inferredBonePen = new Pen(Brushes.Gray, 1);

        private bool disableSkel = false;

        private SwipeGestureDetector swipeGestureRecognizer;
        TemplatedGestureDetector circleGestureRecognizer;


        public MainWindow() {
            InitializeComponent();
            InitializeKinect();
        }

        public void InitializeKinect() {

            this.drawingGroup = new DrawingGroup();
            this.imageSource = new DrawingImage(this.drawingGroup);

            foreach (var potencialSensor in KinectSensor.KinectSensors) {
                if (potencialSensor.Status == KinectStatus.Connected) {
                    this.sensor = potencialSensor;
                    break;
                }
            }

            InitializeScan();
            InitializeSpeech();
        }

        private void InitializeSpeech()
        {

            engine.SetInputToDefaultAudioDevice();
            engine.UpdateRecognizerSetting("CFGConfidenceRejectionThreshold", 70);

            LoadAllGrammars();

            engine.RecognizeAsync(RecognizeMode.Multiple);
            engine.SpeechRecognized += new System.EventHandler<SpeechRecognizedEventArgs>(engineSpeechRecognizer);
        }

        private void InitializeScan() {
            if (this.sensor != null) {

                InitializeColorSensor();
                InitializeDepthSensor();
                InitializeSkeletonSensor();
                InitializeGestures();

                try {
                    this.sensor.Start();
                } catch (IOException) {
                    this.sensor = null;
                }
            }
        }

        // SPEECH
        void engineSpeechRecognizer(object sender, SpeechRecognizedEventArgs e)
        {
            SemanticValue semantics = e.Result.Semantics;
            string rawText = e.Result.Text;

            if (semantics.ContainsKey("action") || semantics.ContainsKey("command"))
            {

                string command = (string)semantics["command"].Value;

                switch (command) {
                    case "Color":
                        this.comboBox.SelectedIndex = 0;
                    break;

                    case "Depth":
                        this.comboBox.SelectedIndex = 1;
                    break;


                    case "Skeleton":
                        this.comboBox.SelectedIndex = 2;
                    break;
                }
            }
        }


        // GRAMMARS 
        private void LoadAllGrammars()
        {
            engine.UnloadAllGrammars();

            Grammar actiosGrammar = ActionsGrammar();
            actiosGrammar.Enabled = true;
            engine.LoadGrammar(actiosGrammar);
        }

        private Grammar ActionsGrammar()
        {
            // Actions Grammar
            var actions = new Choices();
            string[] action_options = new string[] {"Show", "Switch" };
            for (var i = 0; i < action_options.Length; i++)
            {
                SemanticResultValue choiceResultValue = new SemanticResultValue(action_options[i], action_options[i]);
                GrammarBuilder resultValueBuilder = new GrammarBuilder(choiceResultValue);
                actions.Add(resultValueBuilder);
            }
            SemanticResultKey resultKeyActions = new SemanticResultKey("action", actions);
            GrammarBuilder actionsGrammar = new GrammarBuilder(resultKeyActions);

            // Commands Grammar
            var commands = new Choices();
            string[] command_options = new string[] {"Color", "Depth", "Skeleton"};
            for (var i = 0; i < command_options.Length; i++)
            {
                SemanticResultValue choiceResultValue = new SemanticResultValue(command_options[i], command_options[i]);
                GrammarBuilder resultValueBuilder = new GrammarBuilder(choiceResultValue);
                commands.Add(resultValueBuilder);
            }
            SemanticResultKey resultKeyCommands = new SemanticResultKey("command", commands);
            GrammarBuilder commandsGrammar = new GrammarBuilder(resultKeyCommands);
        
            actionsGrammar.Append(commandsGrammar);

            Grammar grammar = new Grammar(actionsGrammar);
            grammar.Name = "Change the select box";
            return grammar;
        }

        // GESTURES
        private void OnGestureDetected(string gesture)
        {

            this.statusText.Text = gesture;

            System.Console.WriteLine(gesture);
            System.Console.WriteLine(this.disableSkel);
            switch (gesture) {
                case "SwipeToLeft":
                    this.comboBox.SelectedIndex = (this.comboBox.SelectedIndex != 0) ? 0 : 1;
                break;

                case "Circle":
                    this.disableSkel = (this.disableSkel == true) ? false : true; 
                break;
            }
        }


        // Draw Methods
        private void Draw() {
            
            switch (this.comboBox.SelectedIndex) {
                case 0:
                case 1:
                    if(this.bitmap != null) {
                        this.imageBox.Source = this.bitmap;
                    }
                break;
                case 2:
                    if (this.imageSource != null) {
                        this.imageBox.Source = this.imageSource;
                    }
                break;
            }
        }

        // Gestures 
        private void InitializeGestures() {
            swipeGestureRecognizer = new SwipeGestureDetector();
            swipeGestureRecognizer.OnGestureDetected += OnGestureDetected;

            using (Stream recordStream = File.Open("../../circleKB.save", FileMode.Open))
            {
                circleGestureRecognizer = new TemplatedGestureDetector("Circle", recordStream);
                circleGestureRecognizer.OnGestureDetected += OnGestureDetected;
            }
        }

        // Color Sensor
        private void InitializeColorSensor() {
            this.sensor.ColorStream.Enable(ColorImageFormat.RgbResolution640x480Fps30);
            this.pixels = new byte[this.sensor.ColorStream.FramePixelDataLength];
            this.bitmap = new WriteableBitmap(this.sensor.ColorStream.FrameWidth, this.sensor.ColorStream.FrameHeight, 96.0, 96.0, PixelFormats.Bgr32, null);
            this.sensor.ColorFrameReady += this.SensorColorFrameReady;
        }

        private void SensorColorFrameReady(object sender, ColorImageFrameReadyEventArgs e) {
            if (this.comboBox.SelectedIndex != 0) return;
            using (ColorImageFrame colorFrame = e.OpenColorImageFrame()) {
                if (colorFrame != null) {
                    colorFrame.CopyPixelDataTo(this.pixels);
                    this.bitmap.WritePixels(new Int32Rect(0, 0, this.bitmap.PixelWidth, this.bitmap.PixelHeight), this.pixels, this.bitmap.PixelWidth * sizeof(int), 0);
                    Draw();
                }
            }
        }

        // Depth Sensor
        private void InitializeDepthSensor() {
            this.sensor.DepthStream.Enable(DepthImageFormat.Resolution640x480Fps30);
            this.pixels = new byte[this.sensor.DepthStream.FramePixelDataLength * sizeof(int)];
            this.depthPixels = new DepthImagePixel[this.sensor.DepthStream.FramePixelDataLength];
            this.bitmap = new WriteableBitmap(this.sensor.DepthStream.FrameWidth, this.sensor.DepthStream.FrameHeight, 96.0, 96.0, PixelFormats.Bgr32, null);
            this.sensor.DepthFrameReady += this.SensorDepthFrameReady;
        }

        private void SensorDepthFrameReady(object sender, DepthImageFrameReadyEventArgs e) {
            if (this.comboBox.SelectedIndex != 1) return;

            using (DepthImageFrame depthFrame = e.OpenDepthImageFrame()) {

                if (depthFrame != null) {
                    depthFrame.CopyDepthImagePixelDataTo(this.depthPixels);

                    int index = 0;
                    int min = depthFrame.MinDepth;
                    int max = depthFrame.MaxDepth;

                    for (var i = 0; i < depthPixels.Length; ++i) {
                        short depth = depthPixels[i].Depth;
                        byte intensity = (byte)(depth >= min && depth <= max ? depth : 0);

                        this.pixels[index++] = intensity;
                        this.pixels[index++] = intensity;
                        this.pixels[index++] = intensity;
                        ++index;
                    }
                    
                    this.bitmap.WritePixels(new Int32Rect(0, 0, this.bitmap.PixelWidth, this.bitmap.PixelHeight), this.pixels, this.bitmap.PixelWidth * sizeof(int), 0);
                    Draw();
                }
            }
        }

        // Skeleton 
        private void InitializeSkeletonSensor() {
            this.sensor.SkeletonStream.Enable();
            this.sensor.SkeletonFrameReady += this.SensorSkeletonFrameReady;
        }

        private void SensorSkeletonFrameReady(object sender, SkeletonFrameReadyEventArgs e) {
            
            Skeleton[] skeletons = new Skeleton[0];

            using (SkeletonFrame skeletonFrame = e.OpenSkeletonFrame()) {
                if (skeletonFrame != null) {
                    skeletons = new Skeleton[skeletonFrame.SkeletonArrayLength];
                    skeletonFrame.CopySkeletonDataTo(skeletons);

                    foreach (Joint joint in skeletons[0].Joints)
                    {
                        if (joint.JointType == JointType.HandLeft)
                        {
                            swipeGestureRecognizer.Add(joint.Position, sensor);
                        }

                        if (joint.JointType == JointType.HandRight)
                        {
                            circleGestureRecognizer.Add(joint.Position, sensor);
                        }
                    }
                }
            }

            if (this.comboBox.SelectedIndex != 2) return;

            using (DrawingContext dc = this.drawingGroup.Open()) {
                // Draw a transparent background to set the render size
                dc.DrawRectangle(Brushes.Black, null, new Rect(0.0, 0.0, RenderWidth, RenderHeight));

                if (skeletons.Length != 0 && this.disableSkel == false)
                {
                    foreach (Skeleton skel in skeletons)
                    {
                        RenderClippedEdges(skel, dc);

                        if (skel.TrackingState == SkeletonTrackingState.Tracked)
                        {
                            DrawBonesAndJoints(skel, dc);
                        }
                        else if (skel.TrackingState == SkeletonTrackingState.PositionOnly)
                        {
                            dc.DrawEllipse(
                            this.centerPointBrush,
                            null,
                            SkeletonPointToScreen(skel.Position),
                            BodyCenterThickness,
                            BodyCenterThickness);
                        }
                    }
                }

                // prevent drawing outside of our render area
                this.drawingGroup.ClipGeometry = new RectangleGeometry(new Rect(0.0, 0.0, RenderWidth, RenderHeight));
            }

            Draw();
        }

        private static void RenderClippedEdges(Skeleton skeleton, DrawingContext drawingContext) {
            if (skeleton.ClippedEdges.HasFlag(FrameEdges.Bottom)) {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, RenderHeight - ClipBoundsThickness, RenderWidth, ClipBoundsThickness));
            }

            if (skeleton.ClippedEdges.HasFlag(FrameEdges.Top))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, 0, RenderWidth, ClipBoundsThickness));
            }

            if (skeleton.ClippedEdges.HasFlag(FrameEdges.Left))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, 0, ClipBoundsThickness, RenderHeight));
            }

            if (skeleton.ClippedEdges.HasFlag(FrameEdges.Right))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(RenderWidth - ClipBoundsThickness, 0, ClipBoundsThickness, RenderHeight));
            }
        }

        private void DrawBonesAndJoints(Skeleton skeleton, DrawingContext drawingContext) {
            // Render Torso
            this.DrawBone(skeleton, drawingContext, JointType.Head, JointType.ShoulderCenter);
            this.DrawBone(skeleton, drawingContext, JointType.ShoulderCenter, JointType.ShoulderLeft);
            this.DrawBone(skeleton, drawingContext, JointType.ShoulderCenter, JointType.ShoulderRight);
            this.DrawBone(skeleton, drawingContext, JointType.ShoulderCenter, JointType.Spine);
            this.DrawBone(skeleton, drawingContext, JointType.Spine, JointType.HipCenter);
            this.DrawBone(skeleton, drawingContext, JointType.HipCenter, JointType.HipLeft);
            this.DrawBone(skeleton, drawingContext, JointType.HipCenter, JointType.HipRight);

            // Left Arm
            this.DrawBone(skeleton, drawingContext, JointType.ShoulderLeft, JointType.ElbowLeft);
            this.DrawBone(skeleton, drawingContext, JointType.ElbowLeft, JointType.WristLeft);
            this.DrawBone(skeleton, drawingContext, JointType.WristLeft, JointType.HandLeft);

            // Right Arm
            this.DrawBone(skeleton, drawingContext, JointType.ShoulderRight, JointType.ElbowRight);
            this.DrawBone(skeleton, drawingContext, JointType.ElbowRight, JointType.WristRight);
            this.DrawBone(skeleton, drawingContext, JointType.WristRight, JointType.HandRight);

            // Left Leg
            this.DrawBone(skeleton, drawingContext, JointType.HipLeft, JointType.KneeLeft);
            this.DrawBone(skeleton, drawingContext, JointType.KneeLeft, JointType.AnkleLeft);
            this.DrawBone(skeleton, drawingContext, JointType.AnkleLeft, JointType.FootLeft);

            // Right Leg
            this.DrawBone(skeleton, drawingContext, JointType.HipRight, JointType.KneeRight);
            this.DrawBone(skeleton, drawingContext, JointType.KneeRight, JointType.AnkleRight);
            this.DrawBone(skeleton, drawingContext, JointType.AnkleRight, JointType.FootRight);


            // ************************************TO DO************************************
            // DIBUJAR TODOS LOS JOINTS COMO ELIPSES (BUCLE)
            // - Si el estado del Joint es "Tracked" usar this.trackedJointBrush;
            // - Si el estado del Joint es "Inferred" usar this.inferredJointBrush;

            Brush drawBrush = null;
            
            if (skeleton.Joints.Count != 0) {
                foreach (Joint joint in skeleton.Joints) {
                    switch (joint.TrackingState) {
                        case JointTrackingState.Tracked:
                            drawBrush = this.trackedJointBrush;
                            break;
                        case JointTrackingState.Inferred:
                            drawBrush = this.inferredJointBrush;
                            break;
                    }
                    drawingContext.DrawEllipse(drawBrush, null, this.SkeletonPointToScreen(joint.Position), JointThickness, JointThickness);
                }
            }

        }

        private Point SkeletonPointToScreen(SkeletonPoint skelpoint) {
            DepthImagePoint depthPoint = this.sensor.CoordinateMapper.MapSkeletonPointToDepthPoint(skelpoint, DepthImageFormat.Resolution640x480Fps30);
            return new Point(depthPoint.X, depthPoint.Y);
        }

        private void DrawBone(Skeleton skeleton, DrawingContext drawingContext, JointType jointType0, JointType jointType1)
        {
            Pen drawPen;

            var joint0 = skeleton.Joints[jointType0];
            var joint1 = skeleton.Joints[jointType1];

            if (joint0.TrackingState == JointTrackingState.Tracked && joint1.TrackingState == JointTrackingState.Tracked) {
                drawPen = this.trackedBonePen;
            } else if (joint0.TrackingState == JointTrackingState.NotTracked || joint1.TrackingState == JointTrackingState.NotTracked) {
                return;
            } else if (joint0.TrackingState == JointTrackingState.Inferred && joint1.TrackingState == JointTrackingState.Inferred) {
                return;
            } else if ((joint0.TrackingState == JointTrackingState.Inferred && joint1.TrackingState == JointTrackingState.Tracked) || (joint0.TrackingState == JointTrackingState.Tracked && joint1.TrackingState == JointTrackingState.Inferred)) {
                drawPen = this.trackedBonePen;
            } else {
                drawPen = this.trackedBonePen;
            }

            drawingContext.DrawLine(drawPen, this.SkeletonPointToScreen(joint0.Position), this.SkeletonPointToScreen(joint1.Position));
        }

    }
}
