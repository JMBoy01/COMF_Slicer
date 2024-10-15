using System;

namespace framework_iiw.Exceptions
{
    internal class ModelLoadException: Exception
    {
        public ModelLoadException(string message) : base(message) {
        }
    }
}
