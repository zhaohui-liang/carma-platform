package gov.dot.fhwa.saxton.glidepath.appcommon;

import gov.dot.fhwa.saxton.glidepath.asd.map.MapMessage;

public class MapMessageDataElement extends DataElement {

	public MapMessageDataElement(MapMessage val) {
		super();
		value_ = val;
	}
	
	public MapMessage value() {
		return value_;
	}
	
	////////////////////////////
	
	protected MapMessage value_;
}
