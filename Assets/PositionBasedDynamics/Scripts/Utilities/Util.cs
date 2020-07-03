using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using Common.Mathematics.LinearAlgebra;

namespace PositionBasedDynamics.Utilities
{
    public class Util
    {
        public double[] NormalizeData(IEnumerable<double> data)
        {
            double dataMax = data.Max();
            double dataMin = data.Min();
            double range = dataMax - dataMin;

            double[] res = (double[])data;
            if(range > 0)
                res = data
                    .Select(d => (d - dataMin) / range)
                    .ToArray();

            return res;
        }

        /*
         * Change particle color according to saturation
         * Note that the value of saturation should be between 0 and 1
         */
        public Vector4d ChangeColor(Vector4d originColor, double saturation)
        {
            Vector3d originRGB = new Vector3d(originColor.x, originColor.y, originColor.z);
            Vector3d saturated = new Vector3d(0.3, 0.1, 0.1);
            Vector3d newRGB;
            //newRGB = originRGB - saturation * originRGB / 10000;
            newRGB = originRGB + saturation * (saturated - originRGB);
            return new Vector4d(newRGB.x, newRGB.y, newRGB.z, originColor.w);
        }
    }
}
