package mypackage.FriendTracker;

import android.content.ContentProvider;
import android.content.ContentValues;
import android.content.Context;
import android.content.Intent;
import android.database.Cursor;
import android.net.Uri;

import java.io.IOException;
import java.util.HashMap;
import java.util.Set;

public class FriendProvider extends ContentProvider {

	private static HashMap<String, String> myFriends;
	private static FriendTrackerControl mService;
	private static Context mContext;
	private static boolean initialized = false;

	public static void initialize(Context context, FriendTrackerControl service) {
		if(!initialized) {
			mService = service;
			mContext = context;
			myFriends = new HashMap<String, String>();
			initialized = true;
		}
	}
	
	public static boolean isInitialized() {
		return initialized;
	}

	public static String getFriend(String id) {
		if (mContext != null)
			return myFriends.get(id);

		return null;
	}

	public static String addFriend(String id, String loc) {
		if (mContext != null)
			return myFriends.put(id, loc);

		return null;
	}

	public static String addFriend(String id) {
		Intent intent = new Intent(mContext,
				mypackage.FriendTracker.FriendTrackerControl.class);
		intent.putExtra("FriendId", id);

		try {
			if (mService != null) {
				mService.lookupFriend(intent);
				return getFriend(id);
			}
			return null;
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
			return null;
		}
	}

	public static String removeFriend(String id) {
		if (mContext != null)
			return myFriends.remove(id);

		return null;
	}

	public static Set<String> getAllFriends() {
		if (mContext != null)
			return myFriends.keySet();

		return null;
	}

	@Override
	public int delete(Uri arg0, String arg1, String[] arg2) {
		// TODO Auto-generated method stub
		return 0;
	}

	@Override
	public String getType(Uri uri) {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public Uri insert(Uri uri, ContentValues values) {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public boolean onCreate() {
		// TODO Auto-generated method stub
		return false;
	}

	@Override
	public Cursor query(Uri uri, String[] projection, String selection,
			String[] selectionArgs, String sortOrder) {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public int update(Uri uri, ContentValues values, String selection,
			String[] selectionArgs) {
		// TODO Auto-generated method stub
		return 0;
	}

}