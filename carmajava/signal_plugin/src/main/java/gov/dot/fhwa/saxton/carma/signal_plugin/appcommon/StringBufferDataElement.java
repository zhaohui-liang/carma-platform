package gov.dot.fhwa.saxton.glidepath.appcommon;

import gov.dot.fhwa.saxton.glidepath.appcommon.DataElement;

public class StringBufferDataElement extends DataElement {

    public StringBufferDataElement(StringBuffer val){
        super();
        value_ = val;
    }

    public void append(String statusMessage)   {
        value_.append(" " + statusMessage);
    }

    public String				value(){
        //returns the value of the data element

        return value_.toString();
    }

    ////////////////////////////////////////////////

    protected StringBuffer value_; //element value
}

