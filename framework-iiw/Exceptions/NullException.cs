using System;

namespace framework_iiw.Exceptions
{
    internal class NullException: Exception
    {
        public NullException(string message) : base(message)
        {
        }
    }
}
