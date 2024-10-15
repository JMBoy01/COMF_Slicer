using System;

namespace framework_iiw.Exceptions
{
    internal class PointNotFoundException: Exception
    {
        public PointNotFoundException(string message) : base(message)
        {
        }
    }
}
