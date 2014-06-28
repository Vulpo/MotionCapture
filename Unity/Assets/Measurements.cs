using System;
using System.Collections;
using System.IO.Ports;
using System.Threading;
using UnityEngine;

namespace DataAcquisition
{
	public class DataAcquisition
	{
		private SerialPort xbee = new SerialPort("COM5", 19200, Parity.None, 8, StopBits.One);
		byte[] serialDataReceived;

		/*class member to avoid having to new() for every measure sample.*/
		Measures measures;
		private bool dataAvailable; //for unity to test

		public DataAcquisition(){
			this.measures.init ();
			serialDataReceived = new byte[Measures.DATASIZE];


			xbee.Open();

			Thread collectionThread = new Thread(new ThreadStart(receiveData));
			collectionThread.Start();
		}

		public bool available(){
			return this.dataAvailable;
		}

		public float[] getCorrectedCurrentAngle(){
			//Debug.Log ("getCorrectedAngle()");
			this.dataAvailable = false;
			float[] angles = {measures.correctedAngleX_1, -measures.correctedAngleY_1, measures.correctedAngleZ_1, 
							  measures.correctedAngleX_2, -measures.correctedAngleY_2, measures.correctedAngleZ_2};
			return angles;
		}

		private void receiveData(){
			while (true) {
				//Debug.Log ("ReadLine()");
				char[] dataChars = (xbee.ReadLine ()).ToCharArray ();
				//Debug.Log (dataChars);
				for (int i=0; i<Measures.DATASIZE; i++) {
						serialDataReceived [i] = (byte)dataChars [i];
				}

				if (isSerialDataReceivedComplete ()) {
					//Debug.Log ("isSerialDataReceivedComplete():true");
					measures.deserialize (serialDataReceived);
					this.dataAvailable = true;
				}

			}
		}

		public bool isSerialDataReceivedComplete()
		{
			return (Array.IndexOf(serialDataReceived, 0) == -1) ? true : false;
		}
	}

	/*Structure so it's set on the heap (faster but always passed by value, not by reference like classes)*/
	struct Measures{
		public const int DATASIZE = 2*6*sizeof(float);
		public float correctedAngleX_1, correctedAngleY_1, correctedAngleZ_1;
		public float correctedAngleX_2, correctedAngleY_2, correctedAngleZ_2;
		byte[] deserializedMeasures;

		public void init(){
			deserializedMeasures = new byte[DATASIZE];
		}

		public void deserialize(byte[] serializedMeasures){
			byte first, second;
			for (int i = 0; i < DATASIZE / 2; i++){
				first = serializedMeasures[2 * i];
				first -= 0x30;
				second = serializedMeasures[(2 * i)+1];
				second -= 0x30;
				first  = (byte) (first << 4);
				deserializedMeasures[i] = (byte) (first | second);
			}
			
			correctedAngleX_1 = BitConverter.ToSingle(deserializedMeasures, 0);
			correctedAngleY_1 = BitConverter.ToSingle(deserializedMeasures, sizeof(float));
			correctedAngleZ_1 = BitConverter.ToSingle(deserializedMeasures, 2 * sizeof(float));
			correctedAngleX_2 = BitConverter.ToSingle(deserializedMeasures, 3 * sizeof(float));
			correctedAngleY_2 = BitConverter.ToSingle(deserializedMeasures, 4 * sizeof(float));
			correctedAngleZ_2 = BitConverter.ToSingle(deserializedMeasures, 5 * sizeof(float));
		}
	};
}
