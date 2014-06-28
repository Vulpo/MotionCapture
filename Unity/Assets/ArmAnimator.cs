using UnityEngine;
using System;
using System.Threading;

public class ArmAnimator : MonoBehaviour {

	public Transform arm1, arm2;

	private DataAcquisition.DataAcquisition dataCollector;
	Thread acquisitionThread;

	// Use this for initialization
	void Start () {
		dataCollector = new DataAcquisition.DataAcquisition ();
		//acquisitionThread = new Thread (new ThreadStart (dataCollector.run));
		//acquisitionThread.Start ();
	}
	
	// Update is called once per frame
	void Update () {
		if (dataCollector.available ()) {
			float[] angles = dataCollector.getCorrectedCurrentAngle();
			arm1.localRotation = Quaternion.Euler (angles[0], angles[1], angles[2]); // (-) because in utity angles are inverted.
			arm2.localRotation = Quaternion.Euler (angles[3]-angles[0], angles[4]-angles[1], angles[5]-angles[2]); // (-) because in utity angles are inverted.
		}
	}
}
